#include QMK_KEYBOARD_H
#include "virtser.h"
#include <process_combo.h>
#include <keymap_steno.h>
#include <process_steno.h>
#include "keyboards/gboards/g/keymap_combo.h"

#define TIMEOUT_ACTIVITY 600000 // 10m
uint32_t time_last_activity;
bool is_led_timeout;
led_flags_t led_state;

void change_led_state(bool is_off) {
    is_led_timeout = is_off;

    if (is_led_timeout) {
        led_state = rgb_matrix_get_flags();
        if (led_state != LED_FLAG_NONE) {
            rgb_matrix_set_flags(LED_FLAG_NONE);
            rgb_matrix_disable_noeeprom();
        }
    } else {
        if (led_state != LED_FLAG_NONE) {
            rgb_matrix_set_flags(led_state);
            rgb_matrix_enable_noeeprom();
        }
    }
}

void keyboard_post_init_user(void) {
    rgb_matrix_enable();
}

// Runs just one time when the keyboard initializes.
void matrix_init_user(void) {
    time_last_activity = timer_read32();

    rgb_matrix_set_flags(LED_FLAG_ALL);
    rgb_matrix_set_color_all(0xFF,  0xFF, 0xFF);
};

// Runs constantly in the background, in a loop.
void matrix_scan_user(void) {
    if (!is_led_timeout && timer_elapsed32(time_last_activity) > TIMEOUT_ACTIVITY) {
        change_led_state(true);
    }
};

#define ST_BOLT QK_STENO_BOLT
#define ST_GEM  QK_STENO_GEMINI

void eeconfig_init_user() {
    steno_set_mode(STENO_MODE_BOLT); // or STENO_MODE_BOLT
}

enum ctrl_keycodes {
    U_T_AUTO = SAFE_RANGE, //USB Extra Port Toggle Auto Detect / Always Active
    U_T_AGCR,              //USB Toggle Automatic GCR control
    DBG_TOG,               //DEBUG Toggle On / Off
    DBG_MTRX,              //DEBUG Toggle Matrix Prints
    DBG_KBD,               //DEBUG Toggle Keyboard Prints
    DBG_MOU,               //DEBUG Toggle Mouse Prints
    MD_BOOT,               //Restart into bootloader after hold timeout
};

enum LAYERS {
    DEFAULT_LAYER = 0,
    NUMPAD_LAYER,
    TOGGLED_SHIFT_LAYER,
    PLOVER_LAYER
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [DEFAULT_LAYER] = LAYOUT(
        KC_ESC,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,             KC_PSCR, KC_SLCK, KC_PAUS, \
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC,   KC_INS,  KC_HOME, KC_PGUP, \
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS,   KC_DEL,  KC_END,  KC_PGDN, \
        KC_BSPC, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_ENT, \
        KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,                              KC_UP, \
        KC_LCTL, KC_LGUI, KC_LALT,                   KC_SPC,                             KC_RALT, TG(2),   KC_RCTL, MO(1) ,            KC_LEFT, KC_DOWN, KC_RGHT \
    ),
    [NUMPAD_LAYER] = LAYOUT(
        _______, _______, _______, _______, _______, KC_MUTE, KC_MEDIA_PLAY_PAUSE, KC_VOLD, KC_VOLU, _______, _______, _______, KC_NUMLOCK,            KC_KP_7, KC_KP_8, KC_KP_9, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,   KC_KP_4, KC_KP_5, KC_KP_6, \
        _______, RGB_SPD, RGB_VAI, RGB_SPI, RGB_HUI, RGB_SAI, _______, U_T_AUTO,U_T_AGCR,_______, _______, _______, _______, _______,   KC_KP_1, KC_KP_2, KC_KP_3, \
        _______, RGB_RMOD,RGB_VAD, RGB_MOD, RGB_HUD, RGB_SAD, _______, _______, _______, _______, _______, _______, _______, \
        _______, RGB_MOD, RGB_TOG, _______, _______, MD_BOOT, NK_TOGG, _______, _______, _______, _______, _______,                              KC_KP_0, \
        _______, _______, _______,                   _______,                            _______, _______, _______, _______,            KC_KP_0, KC_DELETE, KC_KP_ENTER \
    ),
    [TOGGLED_SHIFT_LAYER] = LAYOUT(
        _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______, _______,            _______, _______, _______, \
        _______, LSFT(KC_1), LSFT(KC_2), LSFT(KC_3), LSFT(KC_4), LSFT(KC_5), LSFT(KC_6), LSFT(KC_7), LSFT(KC_8), LSFT(KC_9), LSFT(KC_0), _______, _______, _______,   _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,   _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,                              _______, \
        _______, _______, _______,                   _______,                            _______, _______, _______, _______,            _______, _______, _______ \
        ),
    [PLOVER_LAYER] = LAYOUT(
        XXXXXXX, ST_GEM, ST_BOLT, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,            XXXXXXX, XXXXXXX, XXXXXXX, \
        XXXXXXX, STN_N1,  STN_N2,  STN_N3,  STN_N4,  STN_N5,  STN_N6,  STN_N7,  STN_N8,  STN_N9,  STN_NA,  STN_NB,  STN_NC,  XXXXXXX,   XXXXXXX, XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, STN_S1,  STN_TL,  STN_PL,  STN_HL,  STN_ST1, STN_ST3, STN_FR,  STN_PR,  STN_LR,  STN_TR,  STN_DR,  XXXXXXX,   XXXXXXX, XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, STN_S2,  STN_KL,  STN_WL,  STN_RL,  STN_ST2, STN_ST4, STN_RR,  STN_BR,  STN_GR,  STN_SR,  STN_ZR, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,  STN_A,   STN_O,  XXXXXXX, XXXXXXX, STN_E,   STN_U,   XXXXXXX, XXXXXXX,                              XXXXXXX, \
        _______, _______, _______,                   XXXXXXX,                            XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,            XXXXXXX, XXXXXXX, XXXXXXX \
        ),
    /*
    [X] = LAYOUT(
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,            _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,   _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,   _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
        _______, _______, _______, _______, _______, _______, NK_TOGG, _______, _______, _______, _______, _______,                              _______, \
        _______, _______, _______,                   _______,                            _______, _______, _______, _______,            _______, _______, _______ \
    ),
     [RGB] = LAYOUT(
       ESC: 0,   F1: 1,    F2: 2,    F3: 3,    F4: 4,    F5: 5,    F6: 6,    F7: 7,    F8: 8,    F9: 9,    F10: 10,  F11: 11,  F12: 12,            PSCR: 13, SLCK: 14, PAUS: 15,
       GRV: 16,  1: 17,    2: 18,    3: 19,    4: 20,    5: 21,    6: 22,    7: 23,    8: 24,    9: 25,    0: 26,    MINS: 27, EQL: 28,  BSPC: 29, INS: 30,  HOME: 31, PGUP: 32,
       TAB: 33,  Q: 34,    W: 35,    E: 36,    R: 37,    T: 38,    Y: 39,    U: 40,    I: 41,    O: 42,    P: 43,    LBRC: 44, RBRC: 45, BSLS: 46, DEL: 47,  END: 48,  PGDN: 49,
       CAPS: 50, A: 51,    S: 52,    D: 53,    F: 54,    G: 55,    H: 56,    J: 57,    K: 58,    L: 59,    SCLN: 60, QUOT: 61, ENT: 62,
       LSFT: 63, Z: 64,    X: 65,    C: 66,    V: 67,    B: 68,    N: 69,    M: 70,    COMM: 71, DOT: 72,  SLSH: 73, RSFT: 74,                               UP: 75,
       LCTL: 76, LGUI: 77, LALT: 78,                   SPC: 79,                                  RALT: 80, Fn: 81,   APP: 82,  RCTL: 83,           LEFT: 84, DOWN: 85, RGHT: 86
    ),
    [MATRIX] = LAYOUT(
       0,       1,       2,       3,       4,       5,       6,       7,       8,       9,       10,      11,      12,                 13,      14,      15,
       16,      17,      18,      19,      20,      21,      22,      23,      24,      25,      26,      27,      28,      29,        30,      31,      32,
       33,      34,      35,      36,      37,      38,      39,      40,      41,      42,      43,      44,      45,      46,        47,      48,      49,
       50,      51,      52,      53,      54,      55,      56,      57,      58,      59,      60,      61,      62,
       63,      64,      65,      66,      67,      68,      69,      70,      71,      72,      73,      74,                                   75,
       76,      77,      78,                        79,                                 80,      81,      82,      83,                 84,      85,      86
    ),
    */
};

#ifdef _______
#undef _______
#define _______ {0, 0, 255}
#define ___O___ {15, 255, 255}
#define ___B___ {150, 255, 255}
#define ___x___ {0, 0, 0}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    [DEFAULT_LAYER] = {
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,                            _______,
        _______, _______, _______,                   _______,                            _______, _______, _______, _______,          _______, _______, _______,

        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
    },
    [NUMPAD_LAYER] = {
        ___x___, ___x___, ___x___, ___x___, ___x___, ___O___, ___O___, ___O___, ___O___, ___x___, ___x___, ___x___, ___O___,          ___O___, ___O___, ___O___,
        ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___O___, ___O___, ___O___,
        ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___O___, ___O___, ___O___,
        ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___,
        ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___,                            ___O___,
        ___x___, ___x___, ___x___,                   ___x___,                            ___x___, ___x___, ___x___, ___O___,          ___O___, ___O___, ___O___,

        ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___,
        ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___
    },
    [TOGGLED_SHIFT_LAYER] = {
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,                            _______,
        _______, _______, _______,                   _______,                            _______, _______, _______, _______,          _______, _______, _______,

        ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___,
        ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___, ___O___
    },
    [PLOVER_LAYER] = {
        ___x___, _______, _______, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___, ___x___,          ___x___, ___x___, ___x___,
        ___x___, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, ___x___, ___x___, ___x___, ___x___,
        ___x___, ___x___, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, ___x___, ___x___, ___x___, ___x___,
        ___x___, ___x___, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        ___x___, ___x___, ___x___, ___x___, _______, _______, ___x___, ___x___, _______, _______, ___x___, ___x___,                            ___x___,
        _______, _______, _______,                   ___x___,                            ___x___, ___x___, ___x___, ___x___,          ___x___, ___x___, ___x___,

        ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___,
        ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___, ___B___
    },
//    [_VL] = {
//        PURPLE,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______, _______,
//        _______, _______, _______, TURQ,    _______, _______, _______, _______, TURQ,    _______, _______, _______, _______, _______, _______, _______, _______,
//        _______, AZURE,   AZURE,   AZURE,   PURPLE,  _______, BLUE,    PURPLE,  PURPLE,  PURPLE,  BLUE,    _______, _______, _______, _______, _______, _______,
//        _______, PURPLE,  _______, BLUE,    _______, GOLD,    GOLDEN,  GOLDEN,  GOLDEN,  GOLDEN,  _______, _______, _______,
//        _______, _______, BLUE,    BLUE,    _______, _______, TURQ,    _______, _______, _______, TURQ,    _______,                            _______,
//        _______, _______, _______,                   _______,                            _______, PINK,    _______, _______,          _______, _______, _______
//    },
//    [_YL] = {
//        RED,     _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______, _______,
//        CHART,   BLUE,    BLUE,    BLUE,    BLUE,    BLUE,    BLUE,    BLUE,    BLUE,    BLUE,    BLUE,    _______, _______, _______, _______, PURPLE,  PURPLE,
//        _______, RED,     _______, BLUE,    _______, GOLD,    _______, _______, GREEN,   _______, MAGENT,  _______, GOLD,    GOLD,    _______, PURPLE,  PURPLE,
//        _______, BLUE,    _______, BLUE,    _______, MAGENT,  _______, GREEN,   GREEN,   GREEN,   MAGENT,  _______, _______,
//        _______, ORANGE,  ORANGE,  _______, _______, _______, _______, RED,     MAGENT,  MAGENT,  _______, _______,                            GREEN,
//        _______, _______, _______,                   _______,                   _______, PINK,    _______, _______,                   BLUE,    GREEN,   BLUE
//    },
};

#undef _______
#define _______ KC_TRNS
#endif

void set_layer_color(int layer) {
    for (int i = 0; i < DRIVER_LED_TOTAL; i++) {
        HSV hsv = {
            .h = pgm_read_byte(&ledmap[layer][i][0]),
            .s = pgm_read_byte(&ledmap[layer][i][1]),
            .v = pgm_read_byte(&ledmap[layer][i][2]),
        };
        if (hsv.h || hsv.s || hsv.v) {
            RGB   rgb = hsv_to_rgb(hsv);
            float f   = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
            rgb_matrix_set_color(i, f * rgb.r, f * rgb.g, f * rgb.b);
        } else {
            rgb_matrix_set_color(i, 0,0,0);
        }
    }
}

void rgb_matrix_indicators_user(void) {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE ||
        rgb_matrix_get_flags() == LED_FLAG_UNDERGLOW) {
        return;
    }
    set_layer_color(get_highest_layer(layer_state));
}

#define MODS_SHIFT  (get_mods() & MOD_MASK_SHIFT)
#define MODS_CTRL   (get_mods() & MOD_MASK_CTRL)
#define MODS_ALT    (get_mods() & MOD_MASK_ALT)


bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    static uint32_t key_timer;

    time_last_activity = timer_read32();
    if (is_led_timeout) {
        change_led_state(false);
    }

    switch (keycode) {
        case U_T_AUTO:
            if (record->event.pressed && MODS_SHIFT && MODS_CTRL) {
                TOGGLE_FLAG_AND_PRINT(usb_extra_manual, "USB extra port manual mode");
            }
            return false;
        case U_T_AGCR:
            if (record->event.pressed && MODS_SHIFT && MODS_CTRL) {
                TOGGLE_FLAG_AND_PRINT(usb_gcr_auto, "USB GCR auto mode");
            }
            return false;
        case DBG_TOG:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_enable, "Debug mode");
            }
            return false;
        case DBG_MTRX:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_matrix, "Debug matrix");
            }
            return false;
        case DBG_KBD:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_keyboard, "Debug keyboard");
            }
            return false;
        case DBG_MOU:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_mouse, "Debug mouse");
            }
            return false;
        case MD_BOOT:
            if (record->event.pressed) {
                key_timer = timer_read32();
            } else {
                if (timer_elapsed32(key_timer) >= 500) {
                    reset_keyboard();
                }
            }
            return false;
        case RGB_TOG:
            if (record->event.pressed) {
              switch (rgb_matrix_get_flags()) {
                case LED_FLAG_ALL: {
                    rgb_matrix_set_flags(LED_FLAG_KEYLIGHT | LED_FLAG_MODIFIER | LED_FLAG_INDICATOR);
                    rgb_matrix_set_color_all(0, 0, 0);
                  }
                  break;
                case (LED_FLAG_KEYLIGHT | LED_FLAG_MODIFIER | LED_FLAG_INDICATOR): {
                    rgb_matrix_set_flags(LED_FLAG_UNDERGLOW);
                    rgb_matrix_set_color_all(0, 0, 0);
                  }
                  break;
                case LED_FLAG_UNDERGLOW: {
                    rgb_matrix_set_flags(LED_FLAG_NONE);
                    rgb_matrix_disable_noeeprom();
                  }
                  break;
                default: {
                    rgb_matrix_set_flags(LED_FLAG_ALL);
                    rgb_matrix_enable_noeeprom();
                  }
                  break;
              }
            }
            return false;
        default:
            return true; //Process all other keycodes normally
    }
}


#ifdef VIRTSER_ENABLE
#include "print.h"
/* listen on serial for commands. Either a set of lower case letters mapped to colors,
/  or upper case letters that change RGB mode.
/  special command C takes 3 numbers as arguments, terminated with a newline or comma or excess digits.
Command C takes 3-5octets of RGB settings. Numbers can be terminated with a comma or period.
3 octets = set all LED, 4th argument specfies specfic LED, 4+5 specify start and stop LEDs.
*/

void virtser_recv(uint8_t serIn) {

}

void virtser_send(uint8_t serIn) {
    uprintf("Sent - %c\n", serIn);
}
#endif // VirtSerial