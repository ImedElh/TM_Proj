#include "vl53l1.h"
// vl53l1 sensor headers
#include "vl53l1_platform.h"
#include "vl53l1_api.h"
// GPIO driver
#include <zephyr/drivers/gpio.h>
// I2c driver
#include <zephyr/drivers/i2c.h>

// Register Log module
#define LOG_MODULE_NAME vl53l1
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// Multithreading definition
#define THREAD_SENSOR_PRIORITY  7 
#define THREAD_SENSOR_STACKSIZE 1024
#define THREAD1_PRIORITY        7
#define THREAD1_STACKSIZE       512


K_SEM_DEFINE(instance_monitor_sem, 10, 10);


#define VL53L_GPIO DT_NODELABEL(vl53_int)

// vl53l1x integrated sensor in overlay file
#define I2C0_NODE DT_NODELABEL(vl53l1x)
// Multithreading routines
static void threadSensorMeasurement(void);
static void thread1(void);
// Timer routines
static void sensor_measurement_handler(struct k_timer *timer_id);
// Create measurement timer
K_TIMER_DEFINE(sensor_meas_timer, sensor_measurement_handler, NULL);

static const struct i2c_dt_spec _dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
static const struct gpio_dt_spec vl53_gpio = GPIO_DT_SPEC_GET(VL53L_GPIO, gpios);

static VL53L1_Dev_t _vl53l1Dev;
static VL53L1_RangingMeasurementData_t  _vl53l1RangMesData;

static void init_vl53l1_sensor(void)
{
	VL53L1_Error vl53l1Error;

    // Function allows to ensure that the device is booted and ready.
    vl53l1Error = VL53L1_WaitDeviceBooted(&_vl53l1Dev);
    // Function is called one time, and it performs the device initialization.
    vl53l1Error = VL53L1_DataInit(&_vl53l1Dev);
    // Function allows to load device settings specific for a given use case.
    vl53l1Error = VL53L1_StaticInit(&_vl53l1Dev);
    // Timing budget is the time required by the sensor to perform one range measurement
    vl53l1Error = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&_vl53l1Dev,100000); // minimum and maximum timing budgets are [20ms, 1000ms]
    // When a ranging completes, the device waits for the end of the programmed inter-measurement period before resuming the next ranging
    vl53l1Error = VL53L1_SetInterMeasurementPeriodMilliSeconds(&_vl53l1Dev,1000 ); // sets the inter-measurement period to 1s.

    vl53l1Error = VL53L1_SetDistanceMode(&_vl53l1Dev,VL53L1_DISTANCEMODE_LONG); // short (up to 1.3), medium (up to 3 m), long (up to 4m)
    // Function must be called to start a measurement
    vl53l1Error = VL53L1_StartMeasurement(&_vl53l1Dev);
}

/* STEP 4 - Define the callback function */
void vl53_measured_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
   // VL53L1_Error vl53l1Error;
    //gpio_pin_toggle_dt(&led);
  // vl53l1Error = VL53L1_GetRangingMeasurementData(&_vl53l1Dev, &_vl53l1RangMesData);
     
    //LOG_INF("Measurement in mm = %d\n", _vl53l1RangMesData.RangeMilliMeter);
    //VL53L1_ClearInterruptAndStartMeasurement(&_vl53l1Dev);
    k_sem_give(&instance_monitor_sem);

}
/* STEP 5 - Define a variable of type static struct gpio_callback */
static struct gpio_callback vl53_cb_data;


static void threadSensorMeasurement(void)
{
    VL53L1_Error vl53l1Error;
    int ret;


	if (!device_is_ready(vl53_gpio.port)) {
		return -1;
	}
    ret = gpio_pin_configure_dt(&vl53_gpio, GPIO_INPUT);
	if (ret < 0) {
		return -1;
	}
	/* STEP 3 - Configure the interrupt on the button's pin */
	ret = gpio_pin_interrupt_configure_dt(&vl53_gpio, GPIO_INT_EDGE_TO_ACTIVE );

	/* STEP 6 - Initialize the static struct gpio_callback variable   */
    gpio_init_callback(&vl53_cb_data, vl53_measured_cb, BIT(vl53_gpio.pin)); 	
	
	/* STEP 7 - Add the callback function by calling gpio_add_callback()   */
	gpio_add_callback(vl53_gpio.port, &vl53_cb_data);


    _vl53l1Dev.i2c = &_dev_i2c;
   // I2C communication
	if (!device_is_ready(_dev_i2c.bus)) {
	printk("I2C bus %s is not ready!\n\r",_dev_i2c.bus->name);
	return;
   }
   	// Start vl53l1 sensor measurement timer
	k_timer_start(&sensor_meas_timer, K_SECONDS(5),K_SECONDS(1 * 30));
    
    init_vl53l1_sensor();
	while (1) {
#ifdef SKIP
           // This is a blocking function
            vl53l1Error = VL53L1_WaitMeasurementDataReady(&_vl53l1Dev);
            // Get ranging data
            vl53l1Error = VL53L1_GetRangingMeasurementData(&_vl53l1Dev, &_vl53l1RangMesData);
            LOG_INF("Measurement in mm = %d\n", _vl53l1RangMesData.RangeMilliMeter);
            VL53L1_ClearInterruptAndStartMeasurement(&_vl53l1Dev);
 #endif
          //  k_msleep(1000);
          	k_sem_take(&instance_monitor_sem, K_FOREVER);
            vl53l1Error = VL53L1_GetRangingMeasurementData(&_vl53l1Dev, &_vl53l1RangMesData);
     
       LOG_INF("Measurement in mm = %d\n", _vl53l1RangMesData.RangeMilliMeter);
       VL53L1_ClearInterruptAndStartMeasurement(&_vl53l1Dev);
	}
}

static void thread1(void)
{
	while (1) {
       LOG_INF("Hello, I am thread1\n");
	   k_msleep(10000);
	}
}

static void sensor_measurement_handler(struct k_timer *timer_id)
{
    VL53L1_Error vl53l1Error;
    
    // This is a blocking function
   // vl53l1Error = VL53L1_WaitMeasurementDataReady(&_vl53l1Dev);
    // Get ranging data
   // vl53l1Error = VL53L1_GetRangingMeasurementData(&_vl53l1Dev, &_vl53l1RangMesData);
    LOG_INF("Measurement in mm = %d\n", _vl53l1RangMesData.RangeMilliMeter);
  //  VL53L1_ClearInterruptAndStartMeasurement(&_vl53l1Dev);
    //LOG_INF("Hello, sensor_measurement_handler\n");

}

// Create 2 threads
K_THREAD_DEFINE(threadSensorMeas_id, THREAD_SENSOR_STACKSIZE, threadSensorMeasurement, NULL, NULL, NULL,
		THREAD_SENSOR_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread1_id, THREAD1_STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);
