#include "vl53l1.h"
// vl53l1 sensor headers
#include "vl53l1_platform.h"
#include "vl53l1_api.h"
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

// vl53l1x integrated sensor in overlay file
#define I2C0_NODE DT_NODELABEL(vl53l1x)

static void threadSensorMeasurement(void);
static void thread1(void);

static const struct i2c_dt_spec _dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

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


static void threadSensorMeasurement(void)
{
    VL53L1_Error vl53l1Error;

    _vl53l1Dev.i2c = &_dev_i2c;
   // I2C communication
	if (!device_is_ready(_dev_i2c.bus)) {
	printk("I2C bus %s is not ready!\n\r",_dev_i2c.bus->name);
	return;
   }
   init_vl53l1_sensor();
	while (1) {

           // This is a blocking function
            vl53l1Error = VL53L1_WaitMeasurementDataReady(&_vl53l1Dev);
            // Get ranging data
            vl53l1Error = VL53L1_GetRangingMeasurementData(&_vl53l1Dev, &_vl53l1RangMesData);
            LOG_INF("Measurement in mm = %d\n", _vl53l1RangMesData.RangeMilliMeter);
            VL53L1_ClearInterruptAndStartMeasurement(&_vl53l1Dev);
            k_msleep(1000);
	}
}

static void thread1(void)
{
	while (1) {
       LOG_INF("Hello, I am thread1\n");
	   k_msleep(2000);
	}
}

// Create 2 threads
K_THREAD_DEFINE(threadSensorMeas_id, THREAD_SENSOR_STACKSIZE, threadSensorMeasurement, NULL, NULL, NULL,
		THREAD_SENSOR_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread1_id, THREAD1_STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);