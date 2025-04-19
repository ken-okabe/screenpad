#include <stdio.h>      // printf, fprintf, perror, FILE, fopen, fgets, fclose, fflush
#include <stdlib.h>     // exit, EXIT_FAILURE, EXIT_SUCCESS, malloc, free, abs
#include <string.h>     // strerror, strncmp, strstr, strlen, strcmp, memset
#include <fcntl.h>      // open, O_RDONLY, O_WRONLY, O_NONBLOCK
#include <unistd.h>     // read, write, close, access, F_OK, usleep
#include <errno.h>      // errno
#include <math.h>       // fabs(), round()
#include <sys/time.h>   // gettimeofday, struct timeval
#include <linux/input.h> // struct input_event, EVIOCGRAB
#include <linux/input-event-codes.h> // EV_*, KEY_*, ABS_*, SYN_*, REL_*, BTN_*, ABS_MT_*
#include <linux/uinput.h> // uinput specific definitions
#include <sys/ioctl.h>  // ioctl

// --- Configuration ---
const char *TARGET_DEVICE_NAME = "ILTP7807:00 222A:FFF1";
#define MAX_SLOTS 10 // Max number of touch slots to track

// Single-finger settings
const double SENSITIVITY = 1.2;
const int DEAD_ZONE_THRESHOLD_SQ_MOVE = 10 * 10; // Dead zone for STARTING cursor movement
const int DEAD_ZONE_THRESHOLD_SQ_DRAG_START = 30 * 30; // Larger dead zone for STARTING a drag
const int DEAD_ZONE_THRESHOLD_SQ_TAP_ONE = 20 * 20; // Dead zone for qualifying a single-finger TAP
const long TAP_TIMEOUT_MS_SINGLE = 180;          // Timeout for single-finger tap (LClick)
const long DOUBLE_TAP_TIMEOUT_MS = 300;         // Max interval between taps for double-tap/drag

// Two-finger settings
const int DEAD_ZONE_THRESHOLD_SQ_TAP_TWO = 20 * 20; // Movement threshold for 2-finger tap (RClick)
const long TAP_TIMEOUT_MS_TWO = 200;              // Timeout for 2-finger tap (RClick)

// --- State Structures ---
typedef struct { int active; int tracking_id; int x; int y; int start_x; int start_y; int last_x; int last_y; } SlotState;
typedef struct {
    SlotState slots[MAX_SLOTS]; int current_slot; int active_finger_count;
    int is_moving; int potential_single_tap; int potential_drag_start; int drag_active;
    struct timeval touch_down_time_single; struct timeval last_touch_up_time;
    int potential_two_finger_tap; struct timeval two_finger_touch_time; int two_finger_start_coords_set;
} GestureState;
GestureState gesture_state = {0};

// --- Helper Functions ---
const char* get_event_type_str(unsigned short type){ switch(type){ case EV_SYN: return "EV_SYN"; case EV_KEY: return "EV_KEY"; case EV_REL: return "EV_REL"; case EV_ABS: return "EV_ABS"; case EV_MSC: return "EV_MSC"; case EV_SW: return "EV_SW"; case EV_LED: return "EV_LED"; case EV_SND: return "EV_SND"; case EV_REP: return "EV_REP"; default: return "Unknown Type"; } }
const char* get_code_str(unsigned short type, unsigned short code){ switch(type){ case EV_SYN: switch(code){ case SYN_REPORT: return "SYN_REPORT"; case SYN_CONFIG: return "SYN_CONFIG"; case SYN_MT_REPORT: return "SYN_MT_REPORT"; case SYN_DROPPED: return "SYN_DROPPED"; default: return "SYN_UNKNOWN"; } case EV_KEY: if(code==BTN_TOUCH) return "BTN_TOUCH"; if(code==BTN_LEFT) return "BTN_LEFT"; if(code==BTN_RIGHT) return "BTN_RIGHT"; return "KEY_Code"; case EV_REL: switch(code){ case REL_X: return "REL_X"; case REL_Y: return "REL_Y"; case REL_WHEEL: return "REL_WHEEL"; case REL_HWHEEL: return "REL_HWHEEL"; default: return "REL_UNKNOWN"; } case EV_ABS: switch(code){ case ABS_X: return "ABS_X"; case ABS_Y: return "ABS_Y"; case ABS_MT_SLOT: return "ABS_MT_SLOT"; case ABS_MT_TRACKING_ID: return "ABS_MT_TRACKING_ID"; case ABS_MT_POSITION_X: return "ABS_MT_POSITION_X"; case ABS_MT_POSITION_Y: return "ABS_MT_POSITION_Y"; case ABS_MT_PRESSURE: return "ABS_MT_PRESSURE"; default: return "ABS_UNKNOWN"; } case EV_MSC: switch(code){ case MSC_SCAN: return "MSC_SCAN"; case MSC_SERIAL: return "MSC_SERIAL"; default: return "MSC_UNKNOWN"; } default: return "CODE_UNKNOWN"; } }
char* find_device_path_by_name(const char* targetName){ FILE *fp; char line[256]; char current_name[256] = {0}; char handlers_line[256] = {0}; int found_name_block = 0; char *event_ptr; int event_num = -1; char *device_path = NULL; fp = fopen("/proc/bus/input/devices", "r"); if (fp == NULL) { perror("[ERROR] Cannot open /proc/bus/input/devices"); return NULL; } while (fgets(line, sizeof(line), fp) != NULL) { if (strncmp(line, "N: Name=", 8) == 0) { found_name_block = 0; if (sscanf(line + 8, " \"%[^\"]\"", current_name) == 1 || sscanf(line + 8, "%[^\n]", current_name) == 1) { if (strcmp(current_name, targetName) == 0) { found_name_block = 1; } } } else if (found_name_block && strncmp(line, "H: Handlers=", 12) == 0) { strncpy(handlers_line, line + 12, sizeof(handlers_line) - 1); handlers_line[sizeof(handlers_line) - 1] = '\0'; event_ptr = strstr(handlers_line, "event"); if (event_ptr != NULL) { if (sscanf(event_ptr, "event%d", &event_num) == 1) { break; } } found_name_block = 0; } else if (line[0] == '\n') { found_name_block = 0; } } fclose(fp); if (event_num != -1) { device_path = (char*)malloc(strlen("/dev/input/event") + 10 + 1); if (device_path != NULL) { sprintf(device_path, "/dev/input/event%d", event_num); if (access(device_path, F_OK) == 0) { printf("[INFO] Found device \"%s\" corresponds to path: %s\n", targetName, device_path); return device_path; } else { fprintf(stderr, "[WARN] Found handler 'event%d' for \"%s\", but path %s does not exist or is not accessible.\n", event_num, targetName, device_path); free(device_path); device_path = NULL; } } else { perror("[ERROR] Failed to allocate memory for device path"); } } if (event_num == -1) { fprintf(stderr, "[ERROR] Device with name \"%s\" not found or has no event handler.\n", targetName); } return NULL; }
long timeval_diff_ms(struct timeval *start, struct timeval *end){ return (long)(end->tv_sec - start->tv_sec) * 1000 + (long)(end->tv_usec - start->tv_usec) / 1000;}
// --- uinput Helper Functions ---
int send_uinput_event(int fd, unsigned short type, unsigned short code, int value) { struct input_event ev; memset(&ev, 0, sizeof(ev)); ev.type = type; ev.code = code; ev.value = value; /*printf("      [DEBUG] Sending uinput: type=%u (%s), code=%u (%s), value=%d\n", type, get_event_type_str(type), code, get_code_str(type, code), value);*/ ssize_t n = write(fd, &ev, sizeof(ev)); if (n != sizeof(ev)) { fprintf(stderr, "[ERROR] Failed to write event to uinput device (type:%u code:%u value:%d): %s\n", type, code, value, strerror(errno)); return -1; } return 0; }
int setup_uinput_device() { int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK); if (fd == -1) { perror("[ERROR] Cannot open /dev/uinput"); fprintf(stderr, ">>> Ensure 'uinput' kernel module is loaded and you have write permissions.\n"); return -1; } if (ioctl(fd, UI_SET_EVBIT, EV_REL) == -1) goto error; if (ioctl(fd, UI_SET_EVBIT, EV_KEY) == -1) goto error; if (ioctl(fd, UI_SET_EVBIT, EV_SYN) == -1) goto error; if (ioctl(fd, UI_SET_RELBIT, REL_X) == -1) goto error; if (ioctl(fd, UI_SET_RELBIT, REL_Y) == -1) goto error; if (ioctl(fd, UI_SET_KEYBIT, BTN_LEFT) == -1) goto error; if (ioctl(fd, UI_SET_KEYBIT, BTN_RIGHT) == -1) goto error; struct uinput_user_dev uidev; memset(&uidev, 0, sizeof(uidev)); snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Screenpad Unified Handler"); uidev.id.bustype = BUS_VIRTUAL; uidev.id.vendor  = 0xABCD; uidev.id.product = 0xABCD; uidev.id.version = 1; if (write(fd, &uidev, sizeof(uidev)) != sizeof(uidev)) goto error; if (ioctl(fd, UI_DEV_CREATE) == -1) goto error; printf("[INFO] Created virtual uinput device: %s\n", uidev.name); return fd; error: perror("[ERROR] Failed to setup uinput device via ioctl"); close(fd); return -1; }
void destroy_uinput_device(int fd) { if (fd >= 0) { printf("[INFO] Destroying virtual uinput device...\n"); if (ioctl(fd, UI_DEV_DESTROY) == -1) { fprintf(stderr, "[WARN] Failed to destroy uinput device: %s\n", strerror(errno)); } if (close(fd) == -1) { perror("[WARN] Failed to close uinput device file descriptor"); } } }


// --- Main Function ---
int main() {
    int evdev_fd = -1; int uinput_fd = -1; struct input_event ev; ssize_t n;
    int grab = 1; char *device_path = NULL; int needs_sync = 0;
    int i;

    // Initialize state
    memset(&gesture_state, 0, sizeof(GestureState));
    for (i = 0; i < MAX_SLOTS; ++i) { gesture_state.slots[i].tracking_id = -1; }

    printf("Starting C Unified Touch Handler (V3.10 - Logs Cleaned)...\n"); // Version indication
    printf("!!! This program must be run with root privileges (sudo).\n");
    printf("!!! Touchscreen input will be GRABBED.\n");

    // 1. Find the evdev device path
    device_path = find_device_path_by_name(TARGET_DEVICE_NAME);
    if (device_path == NULL) { return EXIT_FAILURE; }

    // 2. Open and Grab the evdev device
    evdev_fd = open(device_path, O_RDONLY | O_NONBLOCK);
    if (evdev_fd == -1) { fprintf(stderr, "[ERROR] Cannot open evdev device \"%s\": %s\n", device_path, strerror(errno)); goto cleanup; }
    if (ioctl(evdev_fd, EVIOCGRAB, &grab) == -1) { perror("[ERROR] Cannot grab evdev device"); goto cleanup; }
    printf("[INFO] Successfully grabbed evdev device: %s\n", device_path);

    // 3. Setup the virtual uinput device (for Move, LClick, RClick)
    uinput_fd = setup_uinput_device();
    if (uinput_fd == -1) { fprintf(stderr, "[FATAL] Failed to setup uinput device. Exiting.\n"); goto cleanup; }
    printf("[INFO] Waiting 1 second for udev...\n");
    sleep(1);

    printf("[INFO] Ready. 1F Tap=LClick, 1F Swipe=Move, 1F DblTap+Hold+Swipe=Drag, 2F Tap=RClick. Ctrl+C=Exit.\n");

    // 4. Main Event Loop
    while (1) {
        n = read(evdev_fd, &ev, sizeof(struct input_event));
        if (n == (ssize_t)-1) { if (errno == EINTR) continue; if (errno == EWOULDBLOCK) { if (needs_sync) { if(send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0) == 0) { needs_sync = 0; } } usleep(10000); continue; } else { perror("\n[ERROR] Error reading events from evdev device"); break; } }
        else if (n == 0 || n != sizeof(struct input_event)) { fprintf(stderr, "\n[WARN] Read %ld bytes (expected %ld). Ignoring.\n", n, sizeof(struct input_event)); continue; }

        // --- Process Multi-Touch Event ---
        int finger_lifted_slot = -1;
        int previous_finger_count = gesture_state.active_finger_count; // Store count before processing event

        // --- Optional: Log raw event for debugging ---
        // printf("Event: time %ld.%06ld, type %u (%s), code %u (%s), value %d\n",
        //        ev.time.tv_sec, (long)ev.time.tv_usec, ev.type, get_event_type_str(ev.type),
        //        ev.code, get_code_str(ev.type, ev.code), ev.value);


        switch (ev.type) {
            case EV_ABS:
                switch (ev.code) {
                    case ABS_MT_SLOT: if (ev.value >= 0 && ev.value < MAX_SLOTS) { gesture_state.current_slot = ev.value; } break;
                    case ABS_MT_TRACKING_ID:
                        if (gesture_state.current_slot >= 0 && gesture_state.current_slot < MAX_SLOTS) {
                            int current_id = gesture_state.slots[gesture_state.current_slot].tracking_id; int new_id = ev.value;
                            // printf("  [DEBUG] TRACKING_ID: Slot=%d, Value=%d (CurrentID=%d)\n", gesture_state.current_slot, new_id, current_id);
                            if (current_id != -1 && new_id == -1) { // Finger lifted
                                if (gesture_state.slots[gesture_state.current_slot].active) {
                                    finger_lifted_slot = gesture_state.current_slot; // Record which slot lifted
                                    // printf("    [DEBUG] Finger Up: Slot=%d, ID=%d. Active Count was: %d\n", finger_lifted_slot, current_id, previous_finger_count);

                                    // ★★★ Perform Tap/Drag Release Checks HERE ★★★

                                    // --- Two-Finger Tap Check ---
                                    if (gesture_state.potential_two_finger_tap && previous_finger_count == 2) {
                                         // printf("    [2F_TAP_DEBUG] Checking Tap for lifted slot %d\n", finger_lifted_slot);
                                         struct timeval ct; gettimeofday(&ct, NULL); long dur = timeval_diff_ms(&gesture_state.two_finger_touch_time, &ct); int moved = 0;
                                         long long dx_l = (long long)gesture_state.slots[finger_lifted_slot].x - (long long)gesture_state.slots[finger_lifted_slot].start_x; long long dy_l = (long long)gesture_state.slots[finger_lifted_slot].y - (long long)gesture_state.slots[finger_lifted_slot].start_y; if ((dx_l*dx_l + dy_l*dy_l) > DEAD_ZONE_THRESHOLD_SQ_TAP_TWO) { moved = 1; /*printf("      [2F_TAP_DEBUG] Lifted slot moved: dist_sq=%lld\n", (dx_l*dx_l + dy_l*dy_l));*/ }
                                         if (!moved) { for(i=0; i<MAX_SLOTS; ++i) { if(i != finger_lifted_slot && gesture_state.slots[i].active) { long long dx_o = (long long)gesture_state.slots[i].x - (long long)gesture_state.slots[i].start_x; long long dy_o = (long long)gesture_state.slots[i].y - (long long)gesture_state.slots[i].start_y; if ((dx_o*dx_o + dy_o*dy_o) > DEAD_ZONE_THRESHOLD_SQ_TAP_TWO) { moved = 1; /*printf("      [2F_TAP_DEBUG] Other slot %d moved: dist_sq=%lld\n", i, (dx_o*dx_o + dy_o*dy_o));*/ break; } } } }
                                         // printf("      [2F_TAP_DEBUG] Final Check: Duration=%ld ms (Timeout=%ld), Moved=%d\n", dur, TAP_TIMEOUT_MS_TWO, moved);
                                         if (dur < TAP_TIMEOUT_MS_TWO && !moved ) { printf("[INFO] Two-Finger Tap detected! Sending Right Click.\n"); send_uinput_event(uinput_fd, EV_KEY, BTN_RIGHT, 1); send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0); usleep(20000); send_uinput_event(uinput_fd, EV_KEY, BTN_RIGHT, 0); needs_sync = 1; }
                                         gesture_state.potential_two_finger_tap = 0; gesture_state.two_finger_start_coords_set = 0;
                                         // printf("      [DEBUG] Reset 2F flags after check.\n");
                                    }

                                    // --- Single-Finger Tap/Drag Release Check ---
                                    if (previous_finger_count == 1) { // Check if the finger lifted was the *only* finger
                                         // printf("    [1F_TAP_DEBUG] Checking Single Tap/Drag Release for lifted slot %d\n", finger_lifted_slot);
                                         struct timeval current_time; gettimeofday(&current_time, NULL); long duration_ms = timeval_diff_ms(&gesture_state.touch_down_time_single, &current_time);
                                         long long dx_1f = (long long)gesture_state.slots[finger_lifted_slot].x - (long long)gesture_state.slots[finger_lifted_slot].start_x; long long dy_1f = (long long)gesture_state.slots[finger_lifted_slot].y - (long long)gesture_state.slots[finger_lifted_slot].start_y;
                                         int moved_1f = (dx_1f * dx_1f + dy_1f * dy_1f) > DEAD_ZONE_THRESHOLD_SQ_TAP_ONE; // Use TAP_ONE threshold
                                         // printf("      [1F_TAP_DEBUG] Check: PotentialTap=%d, MovedCheck=%d (DistSq=%lld, Thresh=%d), DragActive=%d, Duration=%ld ms\n", gesture_state.potential_single_tap, moved_1f, (dx_1f*dx_1f + dy_1f*dy_1f), DEAD_ZONE_THRESHOLD_SQ_TAP_ONE, gesture_state.drag_active, duration_ms);
                                         if (gesture_state.potential_single_tap && !moved_1f && !gesture_state.drag_active && duration_ms < TAP_TIMEOUT_MS_SINGLE) { printf("[INFO] Single Tap detected! Sending Left Click.\n"); send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 1); send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0); usleep(20000); send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 0); needs_sync = 1; }
                                         else if (gesture_state.drag_active) { printf("[INFO] Drag End (1F). Releasing Left Button.\n"); send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 0); needs_sync = 1; }
                                         // Reset flags after processing lift
                                         gesture_state.potential_single_tap = 0; gesture_state.potential_drag_start = 0; gesture_state.drag_active = 0; gesture_state.is_moving = 0;
                                         gesture_state.last_touch_up_time = current_time; // Record time for double tap check
                                         // printf("      [DEBUG] Reset 1F flags. last_touch_up_time set.\n");
                                    }

                                    // Update state *after* all checks for the lifted finger
                                    gesture_state.slots[finger_lifted_slot].active = 0;
                                    gesture_state.slots[finger_lifted_slot].tracking_id = -1;
                                    gesture_state.active_finger_count--;
                                }
                            } else if (current_id == -1 && new_id != -1) { // New finger down
                                if(gesture_state.current_slot < MAX_SLOTS && !gesture_state.slots[gesture_state.current_slot].active) {
                                    gesture_state.slots[gesture_state.current_slot].active = 1; gesture_state.slots[gesture_state.current_slot].tracking_id = new_id; gesture_state.slots[gesture_state.current_slot].x = 0; gesture_state.slots[gesture_state.current_slot].y = 0; gesture_state.slots[gesture_state.current_slot].start_x = 0; gesture_state.slots[gesture_state.current_slot].start_y = 0; gesture_state.slots[gesture_state.current_slot].last_x = 0; gesture_state.slots[gesture_state.current_slot].last_y = 0; gesture_state.active_finger_count++;
                                    // printf("    [DEBUG] Finger Down: Slot=%d, ID=%d. Active Count: %d\n", gesture_state.current_slot, new_id, gesture_state.active_finger_count);
                                    struct timeval current_time; gettimeofday(&current_time, NULL);
                                    if (gesture_state.active_finger_count == 1) { /*printf("    [DEBUG] State: 1 Finger Down\n");*/ gesture_state.touch_down_time_single = current_time; gesture_state.potential_single_tap = 1; gesture_state.is_moving = 0; gesture_state.drag_active = 0; long time_since_last_up = timeval_diff_ms(&gesture_state.last_touch_up_time, &current_time); if (time_since_last_up < DOUBLE_TAP_TIMEOUT_MS) { gesture_state.potential_drag_start = 1; /*printf("      [DEBUG] Potential Drag Start set (time since up: %ld ms)\n", time_since_last_up);*/ } else { gesture_state.potential_drag_start = 0; } gesture_state.potential_two_finger_tap = 0; gesture_state.two_finger_start_coords_set = 0; }
                                    else if (gesture_state.active_finger_count == 2) { /*printf("    [DEBUG] State: 2 Fingers Down\n");*/ gesture_state.potential_two_finger_tap = 1; gettimeofday(&gesture_state.two_finger_touch_time, NULL); gesture_state.two_finger_start_coords_set = 0; gesture_state.potential_single_tap = 0; gesture_state.potential_drag_start = 0; gesture_state.drag_active = 0; gesture_state.is_moving = 0; }
                                    else { /*printf("    [DEBUG] State: %d Fingers Down - Resetting gestures\n", gesture_state.active_finger_count);*/ gesture_state.potential_single_tap = 0; gesture_state.potential_drag_start = 0; gesture_state.potential_two_finger_tap = 0; gesture_state.drag_active = 0; gesture_state.is_moving = 0; }
                                }
                            }
                        } break; // End ABS_MT_TRACKING_ID
                    case ABS_MT_POSITION_X: if (gesture_state.current_slot >= 0 && gesture_state.current_slot < MAX_SLOTS && gesture_state.slots[gesture_state.current_slot].active) { gesture_state.slots[gesture_state.current_slot].x = ev.value; } break;
                    case ABS_MT_POSITION_Y: if (gesture_state.current_slot >= 0 && gesture_state.current_slot < MAX_SLOTS && gesture_state.slots[gesture_state.current_slot].active) { gesture_state.slots[gesture_state.current_slot].y = ev.value; } break;
                } break; // End EV_ABS

            case EV_SYN:
                if (ev.code == SYN_REPORT) {
                    // printf("  [DEBUG] SYN_REPORT - Active Fingers: %d\n", gesture_state.active_finger_count);
                    int current_active_finger_count = gesture_state.active_finger_count;

                    // Set Start Coords
                    if (gesture_state.potential_two_finger_tap && current_active_finger_count == 2 && !gesture_state.two_finger_start_coords_set) { /*printf("    [DEBUG] Recording 2F start coords on SYN report:\n");*/ for(i=0; i<MAX_SLOTS; ++i) { if(gesture_state.slots[i].active) { gesture_state.slots[i].start_x = gesture_state.slots[i].x; gesture_state.slots[i].start_y = gesture_state.slots[i].y; /*printf("      Slot %d Start: X=%d, Y=%d\n", i, gesture_state.slots[i].start_x, gesture_state.slots[i].start_y);*/ } } gesture_state.two_finger_start_coords_set = 1; }
                    if (current_active_finger_count == 1) { int active_slot = -1; for(i=0; i<MAX_SLOTS; ++i) { if(gesture_state.slots[i].active) { active_slot = i; break; } } if (active_slot != -1 && gesture_state.slots[active_slot].last_x == 0 && gesture_state.slots[active_slot].last_y == 0 && !gesture_state.is_moving && !gesture_state.drag_active) { gesture_state.slots[active_slot].start_x = gesture_state.slots[active_slot].x; gesture_state.slots[active_slot].start_y = gesture_state.slots[active_slot].y; gesture_state.slots[active_slot].last_x = gesture_state.slots[active_slot].x; gesture_state.slots[active_slot].last_y = gesture_state.slots[active_slot].y; /*printf("    [DEBUG] Recording 1F start/last coords: Slot=%d, X=%d, Y=%d\n", active_slot, gesture_state.slots[active_slot].start_x, gesture_state.slots[active_slot].start_y);*/ } }

                    // --- Tap/Drag Release Checks Moved to TRACKING_ID ---

                    // --- Single-Finger Movement Logic ---
                    if (current_active_finger_count == 1) {
                        int active_slot = -1; for(i=0; i<MAX_SLOTS; ++i) { if(gesture_state.slots[i].active) { active_slot = i; break; } }
                        if (active_slot != -1) {
                            long long dist_x = (long long)gesture_state.slots[active_slot].x - (long long)gesture_state.slots[active_slot].start_x; long long dist_y = (long long)gesture_state.slots[active_slot].y - (long long)gesture_state.slots[active_slot].start_y; long long dist_sq = dist_x * dist_x + dist_y * dist_y;
                            int threshold_to_use = gesture_state.potential_drag_start ? DEAD_ZONE_THRESHOLD_SQ_DRAG_START : DEAD_ZONE_THRESHOLD_SQ_MOVE;
                            if (!gesture_state.is_moving && !gesture_state.drag_active && dist_sq > threshold_to_use) {
                                // printf("    [MOVE_DEBUG] Dead zone exceeded (Slot %d): dist_sq=%lld, threshold=%d\n", active_slot, dist_sq, threshold_to_use);
                                gesture_state.is_moving = 1;
                                // Tap potential is checked on lift, not cancelled here
                                if (gesture_state.potential_drag_start) { printf("[INFO] Drag Start (1F DoubleTap+Hold+Swipe)\n"); send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 1); needs_sync = 1; gesture_state.drag_active = 1; gesture_state.potential_drag_start = 0; gesture_state.potential_single_tap = 0; }
                                gesture_state.slots[active_slot].last_x = gesture_state.slots[active_slot].x; gesture_state.slots[active_slot].last_y = gesture_state.slots[active_slot].y;
                            }
                            if (gesture_state.is_moving || gesture_state.drag_active) {
                                int delta_abs_x = gesture_state.slots[active_slot].x - gesture_state.slots[active_slot].last_x; int delta_abs_y = gesture_state.slots[active_slot].y - gesture_state.slots[active_slot].last_y; int dx_rel = 0; int dy_rel = 0;
                                if (delta_abs_x != 0 || delta_abs_y != 0) {
                                     dx_rel = (int)round((double)(delta_abs_y) * SENSITIVITY); dy_rel = (int)round((double)(-delta_abs_x) * SENSITIVITY);
                                     // printf("    [MOVE_DEBUG] Slot %d Delta: dX_abs=%d, dY_abs=%d -> dX_rel=%d, dY_rel=%d\n", active_slot, delta_abs_x, delta_abs_y, dx_rel, dy_rel);
                                     if (dx_rel != 0) { send_uinput_event(uinput_fd, EV_REL, REL_X, dx_rel); needs_sync = 1; }
                                     if (dy_rel != 0) { send_uinput_event(uinput_fd, EV_REL, REL_Y, dy_rel); needs_sync = 1; }
                                     gesture_state.slots[active_slot].last_x = gesture_state.slots[active_slot].x; gesture_state.slots[active_slot].last_y = gesture_state.slots[active_slot].y;
                                }
                            }
                        }
                    }

                    // Send sync to uinput if needed
                    if (needs_sync) { if(send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0) == 0) { needs_sync = 0; } }
                    // Reset potential flags based on current count
                    if(current_active_finger_count != 2 && gesture_state.potential_two_finger_tap) { gesture_state.potential_two_finger_tap = 0; gesture_state.two_finger_start_coords_set = 0; }
                    if(current_active_finger_count != 1 && gesture_state.potential_single_tap) { gesture_state.potential_single_tap = 0;}

                } // end if SYN_REPORT
                break; // End EV_SYN
        } // End switch(ev.type)
    } // End while

cleanup:
    // 5. Cleanup resources
    printf("\n[INFO] Cleaning up...\n");
    if (uinput_fd >= 0 && gesture_state.drag_active) { send_uinput_event(uinput_fd, EV_KEY, BTN_LEFT, 0); send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0); }
    destroy_uinput_device(uinput_fd);
    if (evdev_fd >= 0) { grab = 0; if (ioctl(evdev_fd, EVIOCGRAB, &grab) == -1) { perror("[WARN] Failed to ungrab evdev device"); } else { printf("[INFO] Evdev device ungrabbed.\n"); } if (close(evdev_fd) == -1) { perror("[WARN] Failed to close evdev device file descriptor"); } }
    if (device_path != NULL) { free(device_path); }
    printf("[INFO] Exiting MT handler.\n");

    return (errno == 0 || errno == EINTR) ? EXIT_SUCCESS : EXIT_FAILURE;
}

// --- Full Helper Function Implementations ---
// (Need to paste the full code for get_event_type_str, get_code_str,
//  find_device_path_by_name, timeval_diff_ms, send_uinput_event,
//  setup_uinput_device, destroy_uinput_device here for completeness)
// ... (Ellipsis for brevity in thought, but full code in immersive) ...
