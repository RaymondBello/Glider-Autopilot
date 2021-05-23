#include <Arduino.h>
#include <unity.h>

#ifdef UNIT_TEST

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_led_builtin_pin_number(void) {
    TEST_ASSERT_EQUAL(LED_BUILTIN, 12);
}

void test_led_state_high(void) {
    digitalWrite(LED_BUILTIN, HIGH);
    TEST_ASSERT_EQUAL(digitalRead(LED_BUILTIN), HIGH);
}

void test_led_state_low(void) {
    digitalWrite(LED_BUILTIN, LOW);
    TEST_ASSERT_EQUAL(digitalRead(LED_BUILTIN), LOW);
}

void setup() {
    UNITY_BEGIN();    // IMPORTANT LINE!

    RUN_TEST(test_led_builtin_pin_number);

    pinMode(LED_BUILTIN, OUTPUT);

    uint8_t max_blinks = 5;

    for (int i = 1; i <= max_blinks; i++)
    {
        RUN_TEST(test_led_state_high);
        delay(1000);
        RUN_TEST(test_led_state_low);
        delay(1000);
        RUN_TEST(test_led_builtin_pin_number);
        delay(1000);

        
        if (i == max_blinks) {
        UNITY_END(); // stop unit testing
        }
    }
    Serial.println("Ran All UNIT TESTS!");

    
}


void loop() {
}

#endif