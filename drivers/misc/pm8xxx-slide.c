/*
 * Copyright LG Electronics (c) 2011
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/slide.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <mach/board_lge.h>
#include <linux/sw3800.h>

struct pm8xxx_slide {
	struct switch_dev sdev;
	struct switch_dev detect_sdev;
	struct delayed_work top_work;
	struct delayed_work bottom_work;
	struct delayed_work slide_work;
	struct delayed_work detect_work;
	struct device *dev;
	struct wake_lock wake_lock;
	const struct pm8xxx_slide_platform_data *pdata;
	int top;
	int bottom;
	spinlock_t lock;
	int state;
	int detect;
};

#define SLIDE_DETECT_DELAY 150

static struct workqueue_struct *slide_wq;
static struct pm8xxx_slide *slide;

bool slide_irq_mask = 1;

void slide_enable_irq(unsigned int irq)
{
	if(!slide_irq_mask){
		slide_irq_mask = 1;
		enable_irq(irq);
		printk("[Slide] slide_enable_irq(%d)\n", slide_irq_mask);
	}
}

void slide_disable_irq(unsigned int irq)
{
	if(slide_irq_mask){
		slide_irq_mask = 0;
	disable_irq(irq);
	printk("[Slide] slide_disable_irq(%d)\n", slide_irq_mask);
	}
}

static void boot_slide_det_func(void)
{
	int condition;
	u32 result_t=0;

	if (slide->pdata->backcover_detect_pin) {
		slide->detect = !gpio_get_value(slide->pdata->backcover_detect_pin);

		printk("%s : [Slide] boot detectcover === > %d \n", __func__ , slide->detect);

		switch (slide->detect) {
			case 0:		//BackCover OPEN
				wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
				switch_set_state(&slide->detect_sdev, slide->detect);
				printk("%s : [Slide] detect === > %d\n", __func__ , slide->detect);

				break;

			case 1:		//BackCover Close and Authentication
				result_t = SW3800_Authentication();
				printk("%s : [Slide] SW3800_Authentication === > %d \n", __func__ , result_t);

				wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
				switch_set_state(&slide->detect_sdev, result_t);

				if(result_t==0) {		//Authentication Fail, Disable IRQ
					switch_set_state(&slide->sdev, 0);
					//slide_disable_irq(slide->pdata->hallic_top_irq);
				}
				else {			//Authentication Success, Enable IRQ
					slide->top = !gpio_get_value(slide->pdata->hallic_top_detect_pin);
					printk("%s : [Slide] boot top === > %d \n", __func__ , slide->top);

					if(slide->top == 1)
					condition = SMARTCOVER_SLIDE_CLOSED;
					else
					condition = SMARTCOVER_SLIDE_OPENED;

					printk("%s : [Slide] boot slide value is %d\n", __func__ , condition);

					slide->state = condition;
					wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
					switch_set_state(&slide->sdev, slide->state);
				}
				break;
		}
	}
	else {
		wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&slide->detect_sdev,0);
		switch_set_state(&slide->sdev, 0);
	}
}

static void pm8xxx_top_work_func(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->hallic_top_detect_pin)
		slide->top = !gpio_get_value(slide->pdata->hallic_top_detect_pin);

	spin_unlock_irqrestore(&slide->lock, flags);
	printk("%s : [Slide]top === > %d \n", __func__ , slide->top);

}

static void pm8xxx_bottom_work_func(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->hallic_bottom_detect_pin)
		slide->bottom = !gpio_get_value(slide->pdata->hallic_bottom_detect_pin);

	spin_unlock_irqrestore(&slide->lock, flags);
	printk("%s : [Slide]bottom === > %d \n", __func__ , slide->bottom);
}

static void pm8xxx_slide_work_func(struct work_struct *work)
{
	int slide_state = 0;
	unsigned long flags;
	int tmp_top = 0;
	int tmp_bottom = 0;

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->hallic_top_detect_pin)
		tmp_top = !gpio_get_value(slide->pdata->hallic_top_detect_pin);
	if (slide->pdata->hallic_bottom_detect_pin)
		tmp_bottom = !gpio_get_value(slide->pdata->hallic_bottom_detect_pin);

	 printk("%s : [Slide] tmp_top === > %d \n", __func__ , tmp_top);
	 printk("%s : [Slide] tmp_bottom === > %d \n", __func__ , tmp_bottom);

	if (tmp_top == 1 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_CLOSED;
	else if (tmp_top == 0 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_HALF;
	else
		slide_state = SMARTCOVER_SLIDE_OPENED;


	if (slide->state != slide_state) {
		slide->state = slide_state;
		spin_unlock_irqrestore(&slide->lock, flags);
		wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&slide->sdev, slide->state);
		printk("%s : [Slide] Slide value is %d\n", __func__ , slide_state);
	}
	else {
		spin_unlock_irqrestore(&slide->lock, flags);
		printk("%s : [Slide] Slide value is %d (no change)\n", __func__ , slide_state);
	}
}

static void sw3800_backcover_work_func(struct work_struct *work)
{
	unsigned long flags;
	u32 result_t = 0;

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->backcover_detect_pin){
		slide->detect = !gpio_get_value(slide->pdata->backcover_detect_pin);

		switch (slide->detect) {
			case 0:		//BackCover OPEN
				spin_unlock_irqrestore(&slide->lock, flags);
				wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
				switch_set_state(&slide->detect_sdev, slide->detect);
				printk("%s : [Slide] detect === > %d\n", __func__ , slide->detect);
				break;

			case 1:		//BackCover Close and Authentication
				result_t = SW3800_Authentication();
				printk("%s : [Slide] SW3800_Authentication === > %d \n", __func__ , result_t);

				spin_unlock_irqrestore(&slide->lock, flags);
				wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
				switch_set_state(&slide->detect_sdev, result_t);

				if(result_t==0)		//Authentication Fail, Disable IRQ
					switch_set_state(&slide->sdev, 0);
				else {			//Authentication Success, Enable IRQ
					printk("%s : [Slide] Authentication Success, Enable IRQ\n", __func__ );
				}
				break;
		}
	}
	else {
		spin_unlock_irqrestore(&slide->lock, flags);
		wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&slide->detect_sdev, 0);
		switch_set_state(&slide->sdev, 0);
	}
}

static irqreturn_t pm8xxx_top_irq_handler(int irq, void *handle)
{
	struct pm8xxx_slide *slide_handle = handle;
	int v = 0;
	printk("[Slide] top irq!!!!\n");

	v = 1 + 1*(!gpio_get_value(slide->pdata->hallic_top_detect_pin));
	wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(5));
	queue_delayed_work(slide_wq, &slide_handle->top_work, msecs_to_jiffies(5));
	queue_delayed_work(slide_wq, &slide_handle->slide_work, msecs_to_jiffies(SLIDE_DETECT_DELAY*v+5));
	return IRQ_HANDLED;
}

static irqreturn_t pm8xxx_bottom_irq_handler(int irq, void *handle)
{
	struct pm8xxx_slide *slide_handle = handle;
	int v = 0;
	printk("[Slide] bottom irq!!!!\n");

	v = 1 + 1*(!gpio_get_value(slide->pdata->hallic_bottom_detect_pin));
	wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(5));
	queue_delayed_work(slide_wq, &slide_handle->bottom_work, msecs_to_jiffies(5));
	queue_delayed_work(slide_wq, &slide_handle->slide_work, msecs_to_jiffies(SLIDE_DETECT_DELAY*v+5));
	return IRQ_HANDLED;
}

static irqreturn_t sw3800_backcover_irq_handler(int irq, void *handle)
{
	struct pm8xxx_slide *slide_handle = handle;
	printk("[Slide] backcover irq!!!!\n");

	wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(5));
	queue_delayed_work(slide_wq, &slide_handle->detect_work, msecs_to_jiffies(5));
	return IRQ_HANDLED;
}

static ssize_t
slide_top_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_slide *slide = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "top : %d\n", slide->top);

	return len;
}

static ssize_t
slide_bottom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_slide *slide = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "bottom : %d\n", slide->bottom);

	return len;
}


static ssize_t
slide_slide_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_slide *slide = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "sensing(slide state) : %d\n", slide->state);

	return len;
}


static ssize_t
slide_slide_store(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
       struct pm8xxx_slide *slide = dev_get_drvdata(dev);

       sscanf(buf, "%d\n", &slide->state);
       switch_set_state(&slide->sdev, slide->state);
       return count;
}


static ssize_t
slide_detect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_slide *slide = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "sensing(slide state) : %d\n", slide->detect);

	return len;
}

static ssize_t
slide_detect_store(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
       struct pm8xxx_slide *slide = dev_get_drvdata(dev);

       sscanf(buf, "%d\n", &slide->detect);
       switch_set_state(&slide->detect_sdev, slide->detect);
       return count;
}

static struct device_attribute slide_slide_attr = __ATTR(slide, S_IRUGO | S_IWUSR, slide_slide_show, slide_slide_store);
static struct device_attribute slide_top_attr   = __ATTR(top, S_IRUGO | S_IWUSR, slide_top_show, NULL);
static struct device_attribute slide_bottom_attr   = __ATTR(bottom, S_IRUGO | S_IWUSR, slide_bottom_show, NULL);
static struct device_attribute slide_detect_attr   = __ATTR(detect, S_IRUGO | S_IWUSR, slide_detect_show, slide_detect_store);


static ssize_t slide_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:
		return sprintf(buf, "OPEN\n");
	case 1:
		return sprintf(buf, "CLOSE\n");
	case 2:
		return sprintf(buf, "HALF\n");
	}
	return -EINVAL;
}

static void bu52061nvx_parse_dt(struct device *dev,
		struct pm8xxx_slide_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if ((pdata->hallic_top_detect_pin = of_get_named_gpio_flags(np, "hallic-top-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_top_irq = gpio_to_irq(pdata->hallic_top_detect_pin);

	printk("[Slide] hallic_top_gpio: %d\n", pdata->hallic_top_detect_pin);

	if ((pdata->hallic_bottom_detect_pin = of_get_named_gpio_flags(np, "hallic-bottom-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_bottom_irq = gpio_to_irq(pdata->hallic_bottom_detect_pin);

	printk("[Slide] hallic_bottom_gpio: %d\n", pdata->hallic_bottom_detect_pin);

	if ((pdata->backcover_detect_pin = of_get_named_gpio_flags(np, "cover-detect-irq-gpio", 0, NULL)) > 0)
		pdata->backcover_irq = gpio_to_irq(pdata->backcover_detect_pin);

	printk("[Slide] cover-detect-irq-gpio: %d\n", pdata->backcover_detect_pin);

	if ((pdata->backcover_validation_pin = of_get_named_gpio_flags(np, "cover-validation-gpio", 0, NULL)) > 0)
		SW3800_DETECT = pdata->backcover_validation_pin;

	printk("[Slide] cover-validation-gpio: %d\n", pdata->backcover_validation_pin);

	if ((pdata->cover_pullup_pin = of_get_named_gpio_flags(np, "cover-pullup-gpio", 0, NULL)) > 0)
		SW3800_PULLUP = pdata->cover_pullup_pin;

	printk("[Slide] cover-pullup-gpio: %d\n", pdata->cover_pullup_pin);

	pdata->irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
}

static int __devinit pm8xxx_slide_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int hall_top_gpio_irq = 0, hall_bottom_gpio_irq =0, backcover_gpio_irq=0;

	struct pm8xxx_slide_platform_data *pdata;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct pm8xxx_slide_platform_data),
				GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: [Slide] no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;

		bu52061nvx_parse_dt(&pdev->dev, pdata);

	} else {
		pdata = pdev->dev.platform_data;
	}
	if (!pdata) {
		pr_err("%s: [Slide] no pdata\n", __func__);
		return -ENOMEM;
	}

	slide = kzalloc(sizeof(*slide), GFP_KERNEL);
	if (!slide)
		return -ENOMEM;

	slide->pdata	= pdata;
	slide->sdev.name = "smartcover";
	slide->sdev.print_name = slide_print_name;
	slide->detect_sdev.name = "backcover";
	slide->detect_sdev.print_name = slide_print_name;
	slide->state = 0;
	slide->top = 0;
	slide->bottom = 0;
	slide->detect= 0;

	spin_lock_init(&slide->lock);

	ret = switch_dev_register(&slide->sdev);
	ret = switch_dev_register(&slide->detect_sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	wake_lock_init(&slide->wake_lock, WAKE_LOCK_SUSPEND, "hall_ic_wakeups");

	INIT_DELAYED_WORK(&slide->top_work, pm8xxx_top_work_func);
	INIT_DELAYED_WORK(&slide->bottom_work, pm8xxx_bottom_work_func);
	INIT_DELAYED_WORK(&slide->slide_work, pm8xxx_slide_work_func);
	INIT_DELAYED_WORK(&slide->detect_work, sw3800_backcover_work_func);

	printk("%s : [Slide] init slide\n", __func__);

	/* initialize irq of gpio_hall */
	if (slide->pdata->hallic_top_detect_pin > 0) {
		hall_top_gpio_irq = gpio_to_irq(slide->pdata->hallic_top_detect_pin);
		printk("%s : [Slide] hall_top_gpio_irq = [%d]\n", __func__, hall_top_gpio_irq);
		if (hall_top_gpio_irq < 0) {
			printk("Failed : [Slide] GPIO TO IRQ \n");
			ret = hall_top_gpio_irq;
			goto err_request_irq;
		}

		ret = request_irq(hall_top_gpio_irq, pm8xxx_top_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, slide);
		if (ret > 0) {
			printk(KERN_ERR "%s: [Slide] Can't allocate irq %d, ret %d\n", __func__, hall_top_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(hall_top_gpio_irq) == 0)
			printk("%s :[Slide] enable_irq_wake Enable(1)\n",__func__);
		else
			printk("%s :[Slide] enable_irq_wake failed(1)\n",__func__);
	}

	if (slide->pdata->hallic_bottom_detect_pin > 0) {
		hall_bottom_gpio_irq = gpio_to_irq(slide->pdata->hallic_bottom_detect_pin);
		printk("%s : [Slide] hall_bottom_gpio_irq = [%d]\n", __func__, hall_bottom_gpio_irq);
		if (hall_bottom_gpio_irq < 0) {
			printk("Failed : [Slide] GPIO TO IRQ \n");
			ret = hall_bottom_gpio_irq;
			goto err_request_irq;
		}

		ret = request_irq(hall_bottom_gpio_irq, pm8xxx_bottom_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, slide);
		if (ret > 0) {
			printk(KERN_ERR "%s: [Slide] Can't allocate irq %d, ret %d\n", __func__, hall_bottom_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(hall_bottom_gpio_irq) == 0)
			printk("%s :[Slide] enable_irq_wake Enable(2)\n",__func__);
		else
			printk("%s :[Slide] enable_irq_wake failed(2)\n",__func__);
	}

	if (slide->pdata->backcover_detect_pin > 0) {
		backcover_gpio_irq = gpio_to_irq(slide->pdata->backcover_detect_pin);
		printk("%s : [Slide] hall_bottom_gpio_irq = [%d]\n", __func__, backcover_gpio_irq);
		if (backcover_gpio_irq < 0) {
			printk("Failed : [Slide] GPIO TO IRQ \n");
			ret = backcover_gpio_irq;
			goto err_request_irq;
		}

		ret = request_irq(backcover_gpio_irq, sw3800_backcover_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, slide);
		if (ret > 0) {
			printk(KERN_ERR "%s: [Slide] Can't allocate irq %d, ret %d\n", __func__, backcover_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(backcover_gpio_irq) == 0)
			printk("%s :[Slide] enable_irq_wake Enable(3)\n",__func__);
		else
			printk("%s :[Slide] enable_irq_wake failed(3)\n",__func__);
	}
	printk("%s : [Slide] pdata->irq_flags = [%d]\n", __func__,(int)pdata->irq_flags);

	printk("%s : [Slide] boot_slide_det_func START\n",__func__);
	boot_slide_det_func();

	ret = device_create_file(&pdev->dev, &slide_slide_attr);
	ret = device_create_file(&pdev->dev, &slide_top_attr);
	ret = device_create_file(&pdev->dev, &slide_bottom_attr);
	ret = device_create_file(&pdev->dev, &slide_detect_attr);
	if (ret)
		goto err_request_irq;

	platform_set_drvdata(pdev, slide);
	return 0;

err_request_irq:
	if (hall_top_gpio_irq)
		free_irq(hall_top_gpio_irq, 0);
	if (hall_bottom_gpio_irq)
		free_irq(hall_bottom_gpio_irq, 0);
	if (backcover_gpio_irq)
		free_irq(backcover_gpio_irq, 0);

err_switch_dev_register:
	switch_dev_unregister(&slide->sdev);
	switch_dev_unregister(&slide->detect_sdev);
	kfree(slide);
	return ret;
}

static int __devexit pm8xxx_slide_remove(struct platform_device *pdev)
{
	struct pm8xxx_slide *slide = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&slide->top_work);
	cancel_delayed_work_sync(&slide->bottom_work);
	cancel_delayed_work_sync(&slide->slide_work);
	cancel_delayed_work_sync(&slide->detect_work);
	switch_dev_unregister(&slide->sdev);
	switch_dev_unregister(&slide->detect_sdev);
	platform_set_drvdata(pdev, NULL);
	kfree(slide);

	return 0;
}

static int pm8xxx_slide_suspend(struct device *dev)
{
	return 0;
}

static int pm8xxx_slide_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pm8xxx_slide_pm_ops = {
	.suspend = pm8xxx_slide_suspend,
	.resume = pm8xxx_slide_resume,
};

#ifdef CONFIG_OF
static struct of_device_id bu52061nvx_match_table[] = {
	{ .compatible = "rohm,hall-bu52061nvx", },
	{ },
};
#endif

static struct platform_driver pm8xxx_slide_driver = {
	.probe		= pm8xxx_slide_probe,
	.remove		= __devexit_p(pm8xxx_slide_remove),
	.driver		= {
        	.name    = HALL_IC_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bu52061nvx_match_table,
#endif
#ifdef CONFIG_PM
		.pm	= &pm8xxx_slide_pm_ops,
#endif
	},
};

static int __init pm8xxx_slide_init(void)
{
	slide_wq = create_singlethread_workqueue("slide_wq");
       printk(KERN_ERR "Slide init \n");
	if (!slide_wq)
		return -ENOMEM;
	return platform_driver_register(&pm8xxx_slide_driver);
}
module_init(pm8xxx_slide_init);

static void __exit pm8xxx_slide_exit(void)
{
	if (slide_wq)
		destroy_workqueue(slide_wq);
	platform_driver_unregister(&pm8xxx_slide_driver);
}
module_exit(pm8xxx_slide_exit);

MODULE_ALIAS("platform:" HALL_IC_DEV_NAME);
MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("pm8xxx slide driver");
MODULE_LICENSE("GPL");
