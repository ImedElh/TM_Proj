#include "vl53l1.h"
// vl53l1 sensor headers
#include "vl53l1_platform.h"
#include "vl53l1_api.h"
// GPIO driver
#include <zephyr/drivers/gpio.h>
// I2c driver
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

// Register Log module
#define LOG_MODULE_NAME vl53l1
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// Multithreading definition
#define THREAD_SENSOR_PRIORITY  3
#define THREAD_SENSOR_STACKSIZE 1024
#define THREAD1_PRIORITY        3
#define THREAD1_STACKSIZE       1024
#define WORQ_THREAD_STACK_SIZE  1024
#define WORKQ_PRIORITY          4

// Define stack area used by workqueue thread
static K_THREAD_STACK_DEFINE(my_stack_area, WORQ_THREAD_STACK_SIZE);
// Define queue structure
static struct k_work_q offload_work_q = {0};
/* STEP 7 - Create work_info structure and offload function */
struct work_info {
    struct k_work work;
    char name[25];
} my_work;

// A binary semaphore for vl51 measurement monitor
K_SEM_DEFINE(meas_monitor_sem, 1, 1);
// vl53l1x gpio interrupt pin 
#define VL53L_GPIO DT_NODELABEL(vl53_int)
// vl53l1x integrated sensor in overlay file
#define I2C0_NODE DT_NODELABEL(vl53l1x)
// BLE macros
//#define COMPANY_ID_CODE            0x0059 // Nordics semiconductors
#define COMPANY_ID_CODE             0xFFFF  // This value may be used in the internal and interoperability tests before a Company ID has been assigned. This value shall not be used in shipping end products.
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

//Sensor Measurement Timing
#define VL53_MEAS_TIMING_SEC          120 // 2 minutes

// Multithreading routines
static void threadSensorMeasurement(void);
static void thread1(void);
// workque routine
static void offload_function(struct k_work *work_tem);
// Timer routines
static void sensor_measurement_handler(struct k_timer *timer_id);
// Create measurement timer
K_TIMER_DEFINE(sensor_meas_timer, sensor_measurement_handler, NULL);
// I2C and GPIO devices
static const struct i2c_dt_spec _dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
static const struct gpio_dt_spec vl53_gpio = GPIO_DT_SPEC_GET(VL53L_GPIO, gpios);

// BLE static data
/* STEP 1 - Create an LE Advertising Parameters variable */
static struct bt_le_adv_param *adv_param =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_NONE, /* No options specified*/
			800, /* Min Advertising Interval 500ms (800*0.625ms) */
			801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
			NULL); /* Set to NULL for undirected advertising */
// Specific data
typedef struct adv_mfg_data {
	uint16_t   company_code;	    /* Company Identifier Code. */
	int16_t    RangeMilliMeter;     /* VL53L1 range in mm*/
    uint32_t   temperature;
} adv_mfg_data_type;
static adv_mfg_data_type adv_mfg_data = {COMPANY_ID_CODE,0x00,0x00};
// Advertised data
static const struct bt_data ad[] = {
  //  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA,(unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data))
};
// scan response data
static const struct bt_data sd[] = {
    //BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL),
};

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
    k_sem_give(&meas_monitor_sem);
}
/* STEP 5 - Define a variable of type static struct gpio_callback */
static struct gpio_callback vl53_cb_data;


static void threadSensorMeasurement(void)
{
    VL53L1_Error vl53l1Error;
    int ret;
    temperatureFloat_t  floatTemperature;
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
	k_timer_start(&sensor_meas_timer, K_SECONDS(5),K_SECONDS(1 * VL53_MEAS_TIMING_SEC));
    
    init_vl53l1_sensor();

    /* initialize the work item and connect it to its handler function */ 
	k_work_queue_start(&offload_work_q, my_stack_area,
					K_THREAD_STACK_SIZEOF(my_stack_area), WORKQ_PRIORITY,
					NULL);
	strcpy(my_work.name, "TimerMeasIsr");
	k_work_init(&my_work.work, offload_function);
	while (1) {

          	k_sem_take(&meas_monitor_sem, K_FOREVER);
            vl53l1Error = VL53L1_GetRangingMeasurementData(&_vl53l1Dev, &_vl53l1RangMesData);
            LOG_INF("Measurement in mm = %d\n", _vl53l1RangMesData.RangeMilliMeter);
            // Clear interrupt flag for later measurements
            VL53L1_ClearInterruptAndStartMeasurement(&_vl53l1Dev);
            // Stop measurements (to save power or measurment will be done every 1 sec)
            VL53L1_StopMeasurement(&_vl53l1Dev);
            // Update the adv meas data
            adv_mfg_data.RangeMilliMeter = _vl53l1RangMesData.RangeMilliMeter;
            // Simulate a temperature sensor
            floatTemperature.f = 21.5464; //0x41AC5F07
            memcpy(&adv_mfg_data.temperature,&floatTemperature.byte[0],sizeof(uint32_t));
            LOG_INF("Temp = %02x, %02x, %02x, %02x\n",floatTemperature.byte[3],floatTemperature.byte[2],floatTemperature.byte[1],floatTemperature.byte[0]);
            LOG_INF("Temp = %04x\n",adv_mfg_data.temperature);
            // Update vl53l1 range in mm advertised data
            bt_le_adv_update_data(ad, ARRAY_SIZE(ad),sd, ARRAY_SIZE(sd));
	}
}

static void thread1(void)
{
    int err;
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return;
    }
    LOG_INF("Bluetooth initialized\n");
   // err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
   err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd)); // change advertisement parameters
   
	while (1) {
       LOG_INF("Hello, I am thread1\n");
	   k_msleep(30000);
	}
}

static void sensor_measurement_handler(struct k_timer *timer_id)
{
    // Submit the work queue here to avoid blocking the ISR via a semaphore inside VL53L1_StartMeasurement() routine
    k_work_submit_to_queue(&offload_work_q, &my_work.work);
}


static void offload_function(struct k_work *work_tem)
{
    VL53L1_Error vl53l1Error;
    vl53l1Error = VL53L1_StartMeasurement(&_vl53l1Dev);
    if(vl53l1Error != VL53L1_ERROR_NONE)
    {
      LOG_INF("Measurement not started\n");
    }
    LOG_INF("Measurement Started\n");
}


// Create 2 threads
K_THREAD_DEFINE(threadSensorMeas_id, THREAD_SENSOR_STACKSIZE, threadSensorMeasurement, NULL, NULL, NULL,
		THREAD_SENSOR_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread1_id, THREAD1_STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);
