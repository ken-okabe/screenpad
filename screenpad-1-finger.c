#include <stdio.h>      // printf, fprintf, perror, FILE, fopen, fgets, fclose, fflush
#include <stdlib.h>     // exit, EXIT_FAILURE, EXIT_SUCCESS, malloc, free, labs
#include <string.h>     // strerror, strncmp, strstr, strlen, strcmp, memset
#include <fcntl.h>      // open, O_RDONLY, O_WRONLY, O_NONBLOCK
#include <unistd.h>     // read, write, close, access, F_OK, usleep
#include <errno.h>      // errno
#include <math.h>       // round()
#include <sys/time.h>   // gettimeofday, struct timeval
#include <linux/input.h> // struct input_event, EVIOCGRAB
#include <linux/input-event-codes.h> // EV_*, KEY_*, ABS_*, SYN_*, REL_*, BTN_LEFT
#include <linux/uinput.h> // uinput specific definitions (UI_SET_EVBIT, etc.)
#include <sys/ioctl.h>  // ioctl

// --- Configuration ---
const char *TARGET_DEVICE_NAME = "ILTP7807:00 222A:FFF1";
const double SENSITIVITY = 1.2;
const int DEAD_ZONE_THRESHOLD_SQ = 10 * 10; // Dead zone threshold (squared)
const long TAP_TIMEOUT_MS = 180; // Tap timeout in milliseconds
const long DOUBLE_TAP_TIMEOUT_MS = 300; // Double tap interval threshold

// --- Global State ---
struct TouchState {
    int current_x; int current_y; int start_x; int start_y; int last_sent_x; int last_sent_y;
    int touching; int is_swiping; int initial_x_received; int initial_y_received;
    int start_point_is_valid; struct timeval touch_down_time;
    struct timeval last_touch_up_time; int tap_count; int drag_active;
};
struct TouchState touch_state = {0}; // Initialize all members to 0/NULL

// --- Helper Functions ---
const char* get_event_type_str(unsigned short type) { /* ... (same as before) ... */
    switch (type) {
        case EV_SYN: return "EV_SYN"; case EV_KEY: return "EV_KEY"; case EV_REL: return "EV_REL";
        case EV_ABS: return "EV_ABS"; case EV_MSC: return "EV_MSC"; case EV_SW:  return "EV_SW";
        case EV_LED: return "EV_LED"; case EV_SND: return "EV_SND"; case EV_REP: return "EV_REP";
        default:     return "Unknown Type";
    }
}
const char* get_code_str(unsigned short type, unsigned short code) { /* ... (same as before) ... */
    switch (type) {
        case EV_SYN: switch (code) { case SYN_REPORT: return "SYN_REPORT"; case SYN_CONFIG: return "SYN_CONFIG"; case SYN_MT_REPORT: return "SYN_MT_REPORT"; case SYN_DROPPED: return "SYN_DROPPED"; default: return "SYN_UNKNOWN"; }
        case EV_KEY: if (code == BTN_TOUCH) return "BTN_TOUCH"; if (code == BTN_LEFT) return "BTN_LEFT"; /* ... other keys ... */ return "KEY_Code";
        case EV_REL: switch(code) { case REL_X: return "REL_X"; case REL_Y: return "REL_Y"; case REL_WHEEL: return "REL_WHEEL"; case REL_HWHEEL: return "REL_HWHEEL"; default: return "REL_UNKNOWN"; }
        case EV_ABS: switch (code) { case ABS_X: return "ABS_X"; case ABS_Y: return "ABS_Y"; case ABS_MT_SLOT: return "ABS_MT_SLOT"; case ABS_MT_TRACKING_ID: return "ABS_MT_TRACKING_ID"; case ABS_MT_POSITION_X: return "ABS_MT_POSITION_X"; case ABS_MT_POSITION_Y: return "ABS_MT_POSITION_Y"; case ABS_MT_PRESSURE: return "ABS_MT_PRESSURE"; default: return "ABS_UNKNOWN"; }
        case EV_MSC: switch(code) { case MSC_SCAN: return "MSC_SCAN"; case MSC_SERIAL: return "MSC_SERIAL"; default: return "MSC_UNKNOWN"; }
        default: return "CODE_UNKNOWN";
    }
}

// --- Device Path Finding Function ---
char* find_device_path_by_name(const char* targetName) { /* ... (same as before) ... */
    FILE *fp; char line[256]; char current_name[256] = {0}; char handlers_line[256] = {0};
    int found_name_block = 0; char *event_ptr; int event_num = -1; char *device_path = NULL;
    fp = fopen("/proc/bus/input/devices", "r"); if (fp == NULL) { perror("[ERROR] Cannot open /proc/bus/input/devices"); return NULL; }
    while (fgets(line, sizeof(line), fp) != NULL) {
        if (strncmp(line, "N: Name=", 8) == 0) { found_name_block = 0; if (sscanf(line + 8, " \"%[^\"]\"", current_name) == 1 || sscanf(line + 8, "%[^\n]", current_name) == 1) { if (strcmp(current_name, targetName) == 0) { found_name_block = 1; } } }
        else if (found_name_block && strncmp(line, "H: Handlers=", 12) == 0) { strncpy(handlers_line, line + 12, sizeof(handlers_line) - 1); handlers_line[sizeof(handlers_line) - 1] = '\0'; event_ptr = strstr(handlers_line, "event"); if (event_ptr != NULL) { if (sscanf(event_ptr, "event%d", &event_num) == 1) { break; } } found_name_block = 0; }
        else if (line[0] == '\n') { found_name_block = 0; }
    } fclose(fp);
    if (event_num != -1) { device_path = (char*)malloc(strlen("/dev/input/event") + 10 + 1); if (device_path != NULL) { sprintf(device_path, "/dev/input/event%d", event_num); if (access(device_path, F_OK) == 0) { printf("[INFO] Found device \"%s\" corresponds to path: %s\n", targetName, device_path); return device_path; } else { fprintf(stderr, "[WARN] Found handler 'event%d' for \"%s\", but path %s does not exist or is not accessible.\n", event_num, targetName, device_path); free(device_path); device_path = NULL; } } else { perror("[ERROR] Failed to allocate memory for device path"); } }
    if (event_num == -1) { fprintf(stderr, "[ERROR] Device with name \"%s\" not found or has no event handler.\n", targetName); } return NULL;
}

// --- uinput Helper Functions ---
int send_uinput_event(int fd, unsigned short type, unsigned short code, int value) { /* ... (same as before) ... */
    struct input_event ev; memset(&ev, 0, sizeof(ev)); ev.type = type; ev.code = code; ev.value = value;
    ssize_t n = write(fd, &ev, sizeof(ev)); if (n != sizeof(ev)) { fprintf(stderr, "[ERROR] Failed to write event to uinput device (type:%u code:%u value:%d): %s\n", type, code, value, strerror(errno)); return -1; } return 0;
}
int setup_uinput_device() { /* ... (same as before, V2.1) ... */
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK); if (fd == -1) { perror("[ERROR] Cannot open /dev/uinput"); fprintf(stderr, ">>> Ensure 'uinput' kernel module is loaded and you have write permissions.\n"); return -1; }
    if (ioctl(fd, UI_SET_EVBIT, EV_REL) == -1) goto error; if (ioctl(fd, UI_SET_RELBIT, REL_X) == -1) goto error; if (ioctl(fd, UI_SET_RELBIT, REL_Y) == -1) goto error;
    if (ioctl(fd, UI_SET_EVBIT, EV_SYN) == -1) goto error;
    if (ioctl(fd, UI_SET_EVBIT, EV_KEY) == -1) goto error; if (ioctl(fd, UI_SET_KEYBIT, BTN_LEFT) == -1) goto error;
    struct uinput_user_dev uidev; memset(&uidev, 0, sizeof(uidev)); snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Screenpad Virtual Mouse V2.1"); uidev.id.bustype = BUS_VIRTUAL; uidev.id.vendor  = 0xABCD; uidev.id.product = 0x1234; uidev.id.version = 1;
    if (write(fd, &uidev, sizeof(uidev)) != sizeof(uidev)) goto error;
    if (ioctl(fd, UI_DEV_CREATE) == -1) goto error;
    printf("[INFO] Created virtual uinput device: %s\n", uidev.name); return fd;
error:
    perror("[ERROR] Failed to setup uinput device via ioctl"); close(fd); return -1;
}
void destroy_uinput_device(int fd) { /* ... (same as before) ... */
    if (fd >= 0) { printf("[INFO] Destroying virtual uinput device...\n"); if (ioctl(fd, UI_DEV_DESTROY) == -1) { fprintf(stderr, "[WARN] Failed to destroy uinput device: %s\n", strerror(errno)); } if (close(fd) == -1) { perror("[WARN] Failed to close uinput device file descriptor"); } }
}

// Helper to calculate time difference in milliseconds
long timeval_diff_ms(struct timeval *start, struct timeval *end) {
    return (long)(end->tv_sec - start->tv_sec) * 1000 +
           (long)(end->tv_usec - start->tv_usec) / 1000;
}


// --- Main Function ---
int main() {
    int evdev_fd = -1; int uinput_fd = -1; struct input_event ev; ssize_t n;
    int grab = 1; char *device_path = NULL; int needs_sync = 0;

    printf("Starting C Evdev Mapper (V2.2 - Drag Logic Fix)...\n"); // Version indication
    printf("!!! This program must be run with root privileges (sudo).\n");
    printf("!!! Input events will be GRABBED and not reach the OS.\n");

    // 1. Find the evdev device path
    device_path = find_device_path_by_name(TARGET_DEVICE_NAME);
    if (device_path == NULL) { return EXIT_FAILURE; }

    // 2. Open and Grab the evdev device
    evdev_fd = open(device_path, O_RDONLY | O_NONBLOCK);
    if (evdev_fd == -1) { fprintf(stderr, "[ERROR] Cannot open evdev device \"%s\": %s\n", device_path, strerror(errno)); goto cleanup; }
    if (ioctl(evdev_fd, EVIOCGRAB, &grab) == -1) { perror("[ERROR] Cannot grab evdev device"); goto cleanup; }
    printf("[INFO] Successfully grabbed evdev device: %s\n", device_path);

    // 3. Setup the virtual uinput device
    uinput_fd = setup_uinput_device();
    if (uinput_fd == -1) { fprintf(stderr, "[FATAL] Failed to setup uinput device. Exiting.\n"); goto cleanup; }
    printf("[INFO] Waiting 1 second for udev...\n");
    sleep(1);

    printf("[INFO] Ready. Swipe=Move, Tap=Click, DoubleTap+Hold+Swipe=Drag. Ctrl+C=Exit.\n");

    // 4. Main Event Loop
    while (1) {
        n = read(evdev_fd, &ev, sizeof(struct input_event));

        if (n == (ssize_t)-1) { if (errno == EINTR) continue; if (errno == EWOULDBLOCK) { if (needs_sync) { if(send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0) == 0) { needs_sync = 0; } } usleep(5000); continue; } else { perror("\n[ERROR] Error reading events from evdev device"); break; } }
        else if (n == 0 || n != sizeof(struct input_event)) { fprintf(stderr, "\n[WARN] Read %ld bytes (expected %ld). Ignoring.\n", n, sizeof(struct input_event)); continue; }

        // --- Process the received event ---
        switch (ev.type) {
            case EV_ABS:
                {
                    int current_val = ev.value;
                    int delta_abs = 0;
                    int dx_rel = 0;
                    int dy_rel = 0;

                    if (ev.code == ABS_X) {
                        touch_state.current_x = current_val;
                        if (touch_state.touching) touch_state.initial_x_received = 1;
                    } else if (ev.code == ABS_Y) {
                        touch_state.current_y = current_val;
                        if (touch_state.touching) touch_state.initial_y_received = 1;
                    } else { break; }

                    if (touch_state.touching) {
                        if (!touch_state.start_point_is_valid) {
                            if (touch_state.initial_x_received && touch_state.initial_y_received) {
                                touch_state.start_x = touch_state.current_x;
                                touch_state.start_y = touch_state.current_y;
                                touch_state.last_sent_x = touch_state.current_x;
                                touch_state.last_sent_y = touch_state.current_y;
                                touch_state.start_point_is_valid = 1;
                            }
                        } else { // Start point is valid
                            if (!touch_state.is_swiping) { // Check dead zone only if not already swiping
                                long long dist_x = (long long)touch_state.current_x - (long long)touch_state.start_x;
                                long long dist_y = (long long)touch_state.current_y - (long long)touch_state.start_y;
                                long long dist_sq = dist_x * dist_x + dist_y * dist_y;
                                if (dist_sq > DEAD_ZONE_THRESHOLD_SQ) {
                                    touch_state.is_swiping = 1; // Mark as swiping

                                    // ★ Check if this swipe should start a drag (double tap+hold) ★
                                    if (touch_state.tap_count >= 2 && !touch_state.drag_active) {
                                        printf("[INFO] Drag Start (DoubleTap+Hold+Swipe)\n");
                                        send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 1);
                                        needs_sync = 1;
                                        touch_state.drag_active = 1;
                                    }
                                    // Update last_sent on first move outside deadzone
                                    touch_state.last_sent_x = touch_state.current_x;
                                    touch_state.last_sent_y = touch_state.current_y;
                                }
                            }

                            // ★ If swiping OR dragging, calculate and send relative movement ★
                            if (touch_state.is_swiping || touch_state.drag_active) {
                                delta_abs = 0;
                                if (ev.code == ABS_X) {
                                    delta_abs = touch_state.current_x - touch_state.last_sent_x;
                                } else { // ev.code == ABS_Y
                                    delta_abs = touch_state.current_y - touch_state.last_sent_y;
                                }

                                if (delta_abs != 0) {
                                    // Apply 90-degree clockwise rotation and sensitivity
                                    if (ev.code == ABS_X) {
                                        dy_rel = (int)round((double)(-delta_abs) * SENSITIVITY);
                                        if (dy_rel != 0) {
                                            if (send_uinput_event(uinput_fd, EV_REL, REL_Y, dy_rel) == 0) {
                                                touch_state.last_sent_x = touch_state.current_x;
                                                needs_sync = 1;
                                            }
                                        }
                                    } else { // ev.code == ABS_Y
                                        dx_rel = (int)round((double)(delta_abs) * SENSITIVITY);
                                        if (dx_rel != 0) {
                                            if (send_uinput_event(uinput_fd, EV_REL, REL_X, dx_rel) == 0) {
                                                touch_state.last_sent_y = touch_state.current_y;
                                                needs_sync = 1;
                                            }
                                        }
                                    }
                                } // end if delta_abs != 0
                            } // end if is_swiping or drag_active
                        } // end else (start point is valid)
                    } // end if touching
                } // End of scope for EV_ABS locals
                break; // End of EV_ABS case

            case EV_KEY:
                if (ev.code == BTN_TOUCH) {
                    struct timeval current_time;
                    gettimeofday(&current_time, NULL);

                    if (ev.value == 1) { // Just touched down
                        if (!touch_state.touching) {
                             touch_state.touching = 1;
                             touch_state.is_swiping = 0;
                             touch_state.initial_x_received = 0;
                             touch_state.initial_y_received = 0;
                             touch_state.start_point_is_valid = 0;
                             touch_state.drag_active = 0;
                             touch_state.touch_down_time = current_time;

                             long time_since_last_up = timeval_diff_ms(&touch_state.last_touch_up_time, &current_time);
                             if (time_since_last_up < DOUBLE_TAP_TIMEOUT_MS) {
                                 touch_state.tap_count = 2;
                                 // printf("[DEBUG] Potential double tap (time since last up: %ld ms)\n", time_since_last_up);
                             } else {
                                 touch_state.tap_count = 1;
                             }
                        }
                    } else if (ev.value == 0) { // Just lifted off
                        if (touch_state.touching) {
                             long duration_ms = timeval_diff_ms(&touch_state.touch_down_time, &current_time);

                             if (!touch_state.is_swiping && !touch_state.drag_active && duration_ms < TAP_TIMEOUT_MS) {
                                 printf("[INFO] Tap detected! Sending Left Click (Tap Count: %d).\n", touch_state.tap_count);
                                 send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 1);
                                 send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0);
                                 usleep(20000);
                                 send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 0);
                                 needs_sync = 1;
                             }
                             else if (touch_state.drag_active) {
                                 printf("[INFO] Drag End. Releasing Left Button.\n");
                                 send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 0);
                                 needs_sync = 1;
                             }

                             // ★★★ 修正: tap_count のリセットをここに移動 ★★★
                             touch_state.tap_count = 0; // Reset tap count unconditionally on touch up
                             touch_state.last_touch_up_time = current_time; // Record up time *after* using it

                             // Reset other flags
                             touch_state.touching = 0;
                             touch_state.is_swiping = 0;
                             touch_state.initial_x_received = 0;
                             touch_state.initial_y_received = 0;
                             touch_state.start_point_is_valid = 0;
                             touch_state.drag_active = 0;
                        }
                    }
                }
                break; // End of EV_KEY case

            case EV_SYN:
                if (ev.code == SYN_REPORT) {
                    if (needs_sync) {
                        if(send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0) == 0) {
                             needs_sync = 0;
                        }
                    }
                }
                break; // End of EV_SYN case
        } // End switch
    } // End while

cleanup:
    // 5. Cleanup resources
    printf("\n[INFO] Cleaning up...\n");
    if (uinput_fd >= 0 && touch_state.drag_active) {
         printf("[INFO] Releasing left button on exit...\n");
         send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 0);
         send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0);
    }
    destroy_uinput_device(uinput_fd);
    if (evdev_fd >= 0) { /* ... (Ungrab and close evdev) ... */
        grab = 0; if (ioctl(evdev_fd, EVIOCGRAB, &grab) == -1) { perror("[WARN] Failed to ungrab evdev device"); } else { printf("[INFO] Evdev device ungrabbed.\n"); }
        if (close(evdev_fd) == -1) { perror("[WARN] Failed to close evdev device file descriptor"); }
    }
    if (device_path != NULL) { free(device_path); }
    printf("[INFO] Exiting mapper.\n");

    return (errno == 0 || errno == EINTR) ? EXIT_SUCCESS : EXIT_FAILURE;
}