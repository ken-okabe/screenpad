#include <stdio.h>      // printf, fprintf, perror, FILE, fopen, fgets, fclose, fflush
#include <stdlib.h>     // exit, EXIT_FAILURE, EXIT_SUCCESS, malloc, free, abs
#include <string.h>     // strerror, strncmp, strstr, strlen, strcmp, memset
#include <fcntl.h>      // open, O_RDONLY, O_WRONLY, O_NONBLOCK
#include <unistd.h>     // read, write, close, access, F_OK, usleep
#include <errno.h>      // errno
#include <math.h>       // fabs()
#include <sys/time.h>   // gettimeofday, struct timeval
#include <linux/input.h> // struct input_event, EVIOCGRAB
#include <linux/input-event-codes.h> // EV_*, KEY_*, ABS_*, SYN_*, REL_*, BTN_*, ABS_MT_*
#include <linux/uinput.h> // uinput specific definitions
#include <sys/ioctl.h>  // ioctl

// --- Configuration ---
const char *TARGET_DEVICE_NAME = "ILTP7807:00 222A:FFF1";
const int DEAD_ZONE_THRESHOLD_SQ_TAP = 75 * 75; // Increased dead zone
const long TAP_TIMEOUT_MS = 200; // Two-finger tap timeout (ms)
#define MAX_SLOTS 10 // Max number of touch slots to track

// --- State Structures ---
typedef struct { int active; int tracking_id; int x; int y; int start_x; int start_y; } SlotState;
typedef struct {
    SlotState slots[MAX_SLOTS];
    int current_slot;
    int active_finger_count;
    int potential_two_finger_tap;
    struct timeval two_finger_touch_time;
    int two_finger_start_coords_set; // Flag: Start coords recorded after 2nd finger SYN
} MtState;
MtState mt_state = {0};

// --- Helper Functions ---
const char* get_event_type_str(unsigned short type){ switch(type){ case EV_SYN: return "EV_SYN"; case EV_KEY: return "EV_KEY"; case EV_REL: return "EV_REL"; case EV_ABS: return "EV_ABS"; case EV_MSC: return "EV_MSC"; case EV_SW: return "EV_SW"; case EV_LED: return "EV_LED"; case EV_SND: return "EV_SND"; case EV_REP: return "EV_REP"; default: return "Unknown Type"; } }
const char* get_code_str(unsigned short type, unsigned short code){ switch(type){ case EV_SYN: switch(code){ case SYN_REPORT: return "SYN_REPORT"; case SYN_CONFIG: return "SYN_CONFIG"; case SYN_MT_REPORT: return "SYN_MT_REPORT"; case SYN_DROPPED: return "SYN_DROPPED"; default: return "SYN_UNKNOWN"; } case EV_KEY: if(code==BTN_TOUCH) return "BTN_TOUCH"; if(code==BTN_RIGHT) return "BTN_RIGHT"; return "KEY_Code"; case EV_REL: switch(code){ case REL_X: return "REL_X"; case REL_Y: return "REL_Y"; case REL_WHEEL: return "REL_WHEEL"; case REL_HWHEEL: return "REL_HWHEEL"; default: return "REL_UNKNOWN"; } case EV_ABS: switch(code){ case ABS_X: return "ABS_X"; case ABS_Y: return "ABS_Y"; case ABS_MT_SLOT: return "ABS_MT_SLOT"; case ABS_MT_TRACKING_ID: return "ABS_MT_TRACKING_ID"; case ABS_MT_POSITION_X: return "ABS_MT_POSITION_X"; case ABS_MT_POSITION_Y: return "ABS_MT_POSITION_Y"; case ABS_MT_PRESSURE: return "ABS_MT_PRESSURE"; default: return "ABS_UNKNOWN"; } case EV_MSC: switch(code){ case MSC_SCAN: return "MSC_SCAN"; case MSC_SERIAL: return "MSC_SERIAL"; default: return "MSC_UNKNOWN"; } default: return "CODE_UNKNOWN"; } }
char* find_device_path_by_name(const char* targetName){ FILE *fp; char line[256]; char current_name[256] = {0}; char handlers_line[256] = {0}; int found_name_block = 0; char *event_ptr; int event_num = -1; char *device_path = NULL; /*printf("[DEBUG] Searching for device '%s' in /proc/bus/input/devices...\n", targetName);*/ fp = fopen("/proc/bus/input/devices", "r"); if (fp == NULL) { perror("[ERROR] Cannot open /proc/bus/input/devices"); return NULL; } while (fgets(line, sizeof(line), fp) != NULL) { if (strncmp(line, "N: Name=", 8) == 0) { found_name_block = 0; if (sscanf(line + 8, " \"%[^\"]\"", current_name) == 1 || sscanf(line + 8, "%[^\n]", current_name) == 1) { if (strcmp(current_name, targetName) == 0) { found_name_block = 1; } } } else if (found_name_block && strncmp(line, "H: Handlers=", 12) == 0) { strncpy(handlers_line, line + 12, sizeof(handlers_line) - 1); handlers_line[sizeof(handlers_line) - 1] = '\0'; event_ptr = strstr(handlers_line, "event"); if (event_ptr != NULL) { if (sscanf(event_ptr, "event%d", &event_num) == 1) { /*printf("[DEBUG] Found handlers line: %s -> event%d\n", handlers_line, event_num);*/ break; } } found_name_block = 0; } else if (line[0] == '\n') { found_name_block = 0; } } fclose(fp); if (event_num != -1) { device_path = (char*)malloc(strlen("/dev/input/event") + 10 + 1); if (device_path != NULL) { sprintf(device_path, "/dev/input/event%d", event_num); if (access(device_path, F_OK) == 0) { printf("[INFO] Found device \"%s\" corresponds to path: %s\n", targetName, device_path); return device_path; } else { fprintf(stderr, "[WARN] Found handler 'event%d' for \"%s\", but path %s does not exist or is not accessible.\n", event_num, targetName, device_path); free(device_path); device_path = NULL; } } else { perror("[ERROR] Failed to allocate memory for device path"); } } if (event_num == -1) { fprintf(stderr, "[ERROR] Device with name \"%s\" not found or has no event handler.\n", targetName); } return NULL; }
long timeval_diff_ms(struct timeval *start, struct timeval *end){ return (long)(end->tv_sec - start->tv_sec) * 1000 + (long)(end->tv_usec - start->tv_usec) / 1000;}
// --- uinput Helper Functions ---
int send_uinput_event(int fd, unsigned short type, unsigned short code, int value) { struct input_event ev; memset(&ev, 0, sizeof(ev)); ev.type = type; ev.code = code; ev.value = value; /*printf("      [DEBUG] Sending uinput: type=%u (%s), code=%u (%s), value=%d\n", type, get_event_type_str(type), code, get_code_str(type, code), value);*/ ssize_t n = write(fd, &ev, sizeof(ev)); if (n != sizeof(ev)) { fprintf(stderr, "[ERROR] Failed to write event to uinput device (type:%u code:%u value:%d): %s\n", type, code, value, strerror(errno)); return -1; } return 0; }
int setup_uinput_device() { int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK); if (fd == -1) { perror("[ERROR] Cannot open /dev/uinput"); fprintf(stderr, ">>> Ensure 'uinput' kernel module is loaded and you have write permissions.\n"); return -1; } /*printf("[DEBUG] Opened /dev/uinput (fd=%d)\n", fd);*/ if (ioctl(fd, UI_SET_EVBIT, EV_KEY) == -1) goto error; if (ioctl(fd, UI_SET_EVBIT, EV_SYN) == -1) goto error; if (ioctl(fd, UI_SET_KEYBIT, BTN_RIGHT) == -1) goto error; /*printf("[DEBUG] Enabled uinput events: EV_KEY(BTN_RIGHT), EV_SYN\n");*/ struct uinput_user_dev uidev; memset(&uidev, 0, sizeof(uidev)); snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Screenpad MT Right Clicker"); uidev.id.bustype = BUS_VIRTUAL; uidev.id.vendor  = 0xABCD; uidev.id.product = 0x789A; uidev.id.version = 1; /*printf("[DEBUG] Writing uinput device info...\n");*/ ssize_t wr = write(fd, &uidev, sizeof(uidev)); if (wr != sizeof(uidev)) { fprintf(stderr, "[ERROR] Failed writing uinput device info (ret=%ld): %s\n", wr, strerror(errno)); goto error; } /*printf("[DEBUG] Creating uinput device...\n");*/ if (ioctl(fd, UI_DEV_CREATE) == -1) goto error; printf("[INFO] Created virtual uinput device: %s\n", uidev.name); return fd; error: perror("[ERROR] Failed to setup uinput device via ioctl"); close(fd); return -1; }
void destroy_uinput_device(int fd) { if (fd >= 0) { printf("[INFO] Destroying virtual uinput device...\n"); if (ioctl(fd, UI_DEV_DESTROY) == -1) { fprintf(stderr, "[WARN] Failed to destroy uinput device: %s\n", strerror(errno)); } if (close(fd) == -1) { perror("[WARN] Failed to close uinput device file descriptor"); } } }


// --- Main Function ---
int main() {
    int evdev_fd = -1; int uinput_fd = -1; struct input_event ev; ssize_t n;
    int grab = 1; char *device_path = NULL; int needs_sync = 0;
    int i;

    // Initialize MT state slots
    for (i = 0; i < MAX_SLOTS; ++i) { memset(&mt_state.slots[i], 0, sizeof(SlotState)); mt_state.slots[i].tracking_id = -1; }
    mt_state.current_slot = 0; mt_state.active_finger_count = 0; mt_state.potential_two_finger_tap = 0;
    memset(&mt_state.two_finger_touch_time, 0, sizeof(mt_state.two_finger_touch_time));
    mt_state.two_finger_start_coords_set = 0;

    printf("Starting C Multi-Touch Handler (V2.18 - Tap Logic Final Attempt)...\n"); // Version indication
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

    // 3. Setup the virtual uinput device (for Right Click)
    uinput_fd = setup_uinput_device();
    if (uinput_fd == -1) { fprintf(stderr, "[FATAL] Failed to setup uinput device. Exiting.\n"); goto cleanup; }
    printf("[INFO] Waiting 1 second for udev...\n");
    sleep(1);

    printf("[INFO] Ready. Try tapping with two fingers for right click. Press Ctrl+C=Exit.\n");

    // 4. Main Event Loop
    while (1) {
        n = read(evdev_fd, &ev, sizeof(struct input_event));
        if (n == (ssize_t)-1) { if (errno == EINTR) continue; if (errno == EWOULDBLOCK) { if (needs_sync) { if(send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0) == 0) { needs_sync = 0; } } usleep(10000); continue; } else { perror("\n[ERROR] Error reading events from evdev device"); break; } }
        else if (n == 0 || n != sizeof(struct input_event)) { fprintf(stderr, "\n[WARN] Read %ld bytes (expected %ld). Ignoring.\n", n, sizeof(struct input_event)); continue; }

        // --- Process Multi-Touch Event ---
        // int finger_lifted_in_frame = 0; // Not needed
        int previous_finger_count = mt_state.active_finger_count; // ★ Store count before processing event ★

        switch (ev.type) {
            case EV_ABS:
                switch (ev.code) {
                    case ABS_MT_SLOT: if (ev.value >= 0 && ev.value < MAX_SLOTS) { mt_state.current_slot = ev.value; } break;
                    case ABS_MT_TRACKING_ID:
                        if (mt_state.current_slot >= 0 && mt_state.current_slot < MAX_SLOTS) {
                            int current_id = mt_state.slots[mt_state.current_slot].tracking_id; int new_id = ev.value;
                            if (current_id != -1 && new_id == -1) { // Finger lifted
                                if (mt_state.slots[mt_state.current_slot].active) {
                                    int lifted_slot = mt_state.current_slot;
                                    // ★★★ Perform Tap Check HERE ★★★
                                    // printf("    [TAP_DEBUG] Finger Up: Slot=%d, ID=%d. Active Count was: %d\n", lifted_slot, current_id, previous_finger_count); // Use previous count
                                    // ★ 修正: finger_count_before_event -> previous_finger_count ★
                                    printf("    [TAP_DEBUG] Performing Tap Check: potential=%d, prev_count=%d\n", mt_state.potential_two_finger_tap, previous_finger_count);
                                    if (mt_state.potential_two_finger_tap && previous_finger_count == 2) {
                                         struct timeval ct; gettimeofday(&ct, NULL);
                                         long dur = timeval_diff_ms(&mt_state.two_finger_touch_time, &ct);
                                         int moved = 0;

                                         // Check movement for the lifted finger
                                         long long dx_l = (long long)mt_state.slots[lifted_slot].x - (long long)mt_state.slots[lifted_slot].start_x;
                                         long long dy_l = (long long)mt_state.slots[lifted_slot].y - (long long)mt_state.slots[lifted_slot].start_y;
                                         long long dsq_l = dx_l*dx_l + dy_l*dy_l;
                                         // printf("      [TAP_COORD_DEBUG] Checking tap movement for lifted slot %d:\n", lifted_slot);
                                         // printf("        Start: X=%d, Y=%d\n", mt_state.slots[lifted_slot].start_x, mt_state.slots[lifted_slot].start_y);
                                         // printf("        End:   X=%d, Y=%d\n", mt_state.slots[lifted_slot].x, mt_state.slots[lifted_slot].y);
                                         // printf("        DistSq: %lld (Threshold=%d)\n", dsq_l, DEAD_ZONE_THRESHOLD_SQ_TAP);
                                         if (dsq_l > DEAD_ZONE_THRESHOLD_SQ_TAP) { moved = 1; }

                                         // Check movement for the other active finger
                                         if (!moved) {
                                             for(i=0; i<MAX_SLOTS; ++i) {
                                                 if(i != lifted_slot && mt_state.slots[i].active) {
                                                      long long dx_o = (long long)mt_state.slots[i].x - (long long)mt_state.slots[i].start_x;
                                                      long long dy_o = (long long)mt_state.slots[i].y - (long long)mt_state.slots[i].start_y;
                                                      long long dsq_o = dx_o*dx_o + dy_o*dy_o;
                                                      // printf("      [TAP_COORD_DEBUG] Checking tap movement for other active slot %d:\n", i);
                                                      // printf("        Start: X=%d, Y=%d\n", mt_state.slots[i].start_x, mt_state.slots[i].start_y);
                                                      // printf("        End:   X=%d, Y=%d\n", mt_state.slots[i].x, mt_state.slots[i].y);
                                                      // printf("        DistSq: %lld (Threshold=%d)\n", dsq_o, DEAD_ZONE_THRESHOLD_SQ_TAP);
                                                      if (dsq_o > DEAD_ZONE_THRESHOLD_SQ_TAP) { moved = 1; break; }
                                                 }
                                             }
                                         }

                                         // printf("      [TAP_DEBUG] Final Tap Result: Duration=%ld ms (Timeout=%ld), Moved=%d\n", dur, TAP_TIMEOUT_MS, moved);
                                         if (dur < TAP_TIMEOUT_MS && !moved ) { // Movement check re-enabled
                                             printf("[INFO] Two-Finger Tap detected! Sending Right Click.\n");
                                             send_uinput_event(uinput_fd, EV_KEY, BTN_RIGHT, 1); send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0);
                                             usleep(20000);
                                             send_uinput_event(uinput_fd, EV_KEY, BTN_RIGHT, 0); needs_sync = 1;
                                         }
                                         mt_state.potential_two_finger_tap = 0; // Reset tap potential after check
                                         mt_state.two_finger_start_coords_set = 0; // Reset start coord flag
                                    }
                                    // Update state *after* check
                                    mt_state.slots[lifted_slot].active = 0;
                                    mt_state.slots[lifted_slot].tracking_id = -1;
                                    mt_state.active_finger_count--; // Decrement count *after* using previous_finger_count
                                }
                            } else if (current_id == -1 && new_id != -1) { // New finger down
                                if(mt_state.current_slot < MAX_SLOTS && !mt_state.slots[mt_state.current_slot].active) {
                                    mt_state.slots[mt_state.current_slot].active = 1;
                                    mt_state.slots[mt_state.current_slot].tracking_id = new_id;
                                    mt_state.slots[mt_state.current_slot].x = 0; mt_state.slots[mt_state.current_slot].y = 0; mt_state.slots[mt_state.current_slot].start_x = 0; mt_state.slots[mt_state.current_slot].start_y = 0;
                                    mt_state.active_finger_count++;
                                    if (mt_state.active_finger_count == 2) { mt_state.potential_two_finger_tap = 1; gettimeofday(&mt_state.two_finger_touch_time, NULL); mt_state.two_finger_start_coords_set = 0; }
                                    else { if (mt_state.potential_two_finger_tap) { mt_state.potential_two_finger_tap = 0; } }
                                }
                            }
                        }
                        break; // End ABS_MT_TRACKING_ID
                    case ABS_MT_POSITION_X: if (mt_state.current_slot >= 0 && mt_state.current_slot < MAX_SLOTS && mt_state.slots[mt_state.current_slot].active) { mt_state.slots[mt_state.current_slot].x = ev.value; } break;
                    case ABS_MT_POSITION_Y: if (mt_state.current_slot >= 0 && mt_state.current_slot < MAX_SLOTS && mt_state.slots[mt_state.current_slot].active) { mt_state.slots[mt_state.current_slot].y = ev.value; } break;
                }
                break; // End EV_ABS

            case EV_SYN:
                if (ev.code == SYN_REPORT) {
                    if (mt_state.potential_two_finger_tap && mt_state.active_finger_count == 2 && !mt_state.two_finger_start_coords_set) {
                        // printf("    [TAP_DEBUG] Recording start coords on SYN report:\n");
                        for(i=0; i<MAX_SLOTS; ++i) { if(mt_state.slots[i].active) { mt_state.slots[i].start_x = mt_state.slots[i].x; mt_state.slots[i].start_y = mt_state.slots[i].y; /*printf("      Slot %d Start: X=%d, Y=%d\n", i, mt_state.slots[i].start_x, mt_state.slots[i].start_y);*/ } }
                        mt_state.two_finger_start_coords_set = 1;
                    }
                    if (needs_sync) { if(send_uinput_event(uinput_fd, EV_SYN, SYN_REPORT, 0) == 0) { needs_sync = 0; } }
                    if(mt_state.active_finger_count != 2 && mt_state.potential_two_finger_tap) { mt_state.potential_two_finger_tap = 0; mt_state.two_finger_start_coords_set = 0; }
                } // end if SYN_REPORT
                break; // End EV_SYN
        } // End switch(ev.type)
    } // End while

cleanup:
    // 5. Cleanup resources
    printf("\n[INFO] Cleaning up...\n");
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
