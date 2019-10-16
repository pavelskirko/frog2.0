#ifndef BUTTON_H_
#define BUTTON_H_


#define PIN_BIT(x) (1ULL<<x)

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)

TaskHandle_t button_task_handle;

typedef struct {
	uint8_t pin;
    uint8_t event;
} button_event_t;

QueueHandle_t * button_init(unsigned long long pin_select);


#endif
