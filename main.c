#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_midi_host.h"
#include "button.h"

#define DEBUG false
const uint8_t NUM_BUTTONS = 8;
const uint8_t dataSize = 146;

const uint8_t led_pins[8] = {10, 11, 12, 13, 14, 15, 26, 27};
const uint8_t effectState_idx[6] = {6, 26, 47, 67, 88, 108};
uint8_t syx_indetity_request[6] = {0xf0, 0x7e, 0x00, 0x06, 0x01, 0xf7};
uint8_t syx_parameter_edit_enable[6] = {0xf0, 0x52, 0x00, 0x61, 0x50, 0xf7};
uint8_t syx_parameter_edit_disable[6] = {0xf0, 0x52, 0x00, 0x61, 0x51, 0xf7};
uint8_t syx_request_current_program[6] = {0xf0, 0x52, 0x00, 0x61, 0x33, 0xf7};
uint8_t syx_request_current_patch[6] = {0xf0, 0x52, 0x00, 0x61, 0x29, 0xf7};

// [5] = effect#(0-2), [7] = 0:Off, 1:On
uint8_t syx_request_toggle_effect[10] = {0xf0, 0x52, 0x00, 0x61, 0x31, 0x00, 0x00, 0x00, 0x00, 0xf7};

// [5]=effect#, [6]=param#+2, [7]=value. value range is depends on each effect.
uint8_t syx_parameter_edit[10] = {0xf0, 0x52, 0x00, 0x61, 0x31, 0x00, 0x00, 0x00, 0x00, 0xf7};

typedef enum
{
  LOW,
  HIGH
} led_state_t;

typedef enum
{
  EMPTY,
  READING,
  COMPLETE
} message_state_t;

typedef enum
{
  IDLE,
  WAITING,
  BUSY
} core_state_t;

typedef enum
{
  DISCONNECTED,
  CONNECTED,
  READY,
  IDENTIFIED,
  IDENTIFIED_NOT_SUPPORTED,
  EDITOR_ON
} device_state_t;

typedef enum
{
  NONE,
  IDENTIFY_REQUEST,
  IDENTIFY_PENDING,
  IDENTIFY_RECEIVED,
  EDITOR_ON_REQUEST,
  EDITOR_ON_PENDING,
  EDITOR_ON_RECEIVED,
  CURRENT_PATCH_REQUEST,
  CURRENT_PATCH_PENDING,
  CURRENT_PATCH_RECEIVED,
  CURRENT_PATCH_SEND,
  SET_PARAM_REQUEST,
  SET_PARAM_PENDING,
  SET_PARAM_RECEIVED,
  TOGGLE_EFFECT_REQUEST,
  TOGGLE_EFFECT_PENDING,
  TOGGLE_EFFECT_RECEIVED,
  PROGRAM_CHANGE_REQUEST,
  PROGRAM_CHANGE_PENDING,
  PROGRAM_CHANGE_RECEIVED,
  CURRENT_PROGRAM_REQUEST,
  TOGGLE_EFFECT_STATE
} core_task_t;

typedef struct
{
  uint8_t message[146];
  int size;
  message_state_t status;
} sysex_message_t;

typedef struct
{
  uint8_t program;
  uint8_t channel;
  message_state_t status;
} pc_message_t;

typedef struct
{
  uint8_t addr;
  device_state_t status;
  uint8_t program;
  uint8_t effect;
  uint8_t maxEffects;
  uint8_t patchData[146];
} midi_device_t;

sysex_message_t sysex = {
    .message = {0},
    .size = 0,
    .status = EMPTY};

pc_message_t pc_message = {
    .program = 0,
    .channel = 0,
    .status = EMPTY};

midi_device_t ms70cdr = {
    .addr = 0,
    .status = DISCONNECTED,
    .program = 0,
    .effect = 0,
    .maxEffects = 0,
    .patchData = {0}};

// holding device descriptor
tusb_desc_device_t desc_device;

led_state_t led_states[8] = {LOW};
bool sysex_complete = false;

static bool device_mounted = false;
core_state_t core0_state = IDLE;
core_task_t core0_task = NONE;
core_task_t new_task = NONE;

void handle_states_and_tasks();
static void midi_host_task(void);
void clone_descriptors(tuh_xfer_t *xfer);
void send_sysex(uint8_t *message, int size);
void send_program_change(uint8_t program, uint8_t channel);
static void writeLEDs();
static void btn_onchange(button_t *button_p);
void SetCurrentProgram(uint8_t program);
void SetCurrentEffect(uint8_t n);
void ToggleEffect(uint8_t n);
void ToggleEffectState(uint8_t n);
void print_buffer(uint8_t *data, uint8_t size, char *message);

int main()
{
  // set_sys_clock_khz(120000, true);
  bi_decl(bi_program_description("Ziepos MS70CDR MIDI Controller"));
  if (DEBUG)
    printf("Ziepos MS70CDR MIDI Controller\r\n");
  board_init();
  tusb_init();

  // Initialize LED pin
  for (uint8_t i = 0; i < sizeof(led_pins); i++)
  {
    gpio_init(led_pins[i]);
    gpio_set_dir(led_pins[i], GPIO_OUT);
  }

  // Initialize the buttons
  button_t *button1 = create_button(2, btn_onchange);
  button_t *button2 = create_button(3, btn_onchange);
  button_t *button3 = create_button(4, btn_onchange);
  button_t *button4 = create_button(5, btn_onchange);
  button_t *button5 = create_button(6, btn_onchange);
  button_t *button6 = create_button(7, btn_onchange);
  button_t *button7 = create_button(8, btn_onchange);
  button_t *button8 = create_button(9, btn_onchange);

  while (1)
  {
    tuh_task();
    midi_host_task();
    handle_states_and_tasks();
    writeLEDs();
  }
}

//--------------------------------------------------------------------+
// MS70CDR Controller
//--------------------------------------------------------------------+

static void writeLEDs()
{
  for (uint8_t i = 0; i < NUM_BUTTONS; i++)
  {
    gpio_put(led_pins[i], led_states[i]);
  }
}

static void btn_onchange(button_t *button_p)
{
  button_t *button = (button_t *)button_p;
  if (!button->state)
    return; // Ignore button press. Invert the logic if using a pullup (internal or external).
  if (core0_state == WAITING)
  {
    if (core0_task == NONE)
    {
      if (ms70cdr.status == IDENTIFIED)
      {
        if (button->pin == 2)
        {
          // new_task = PROGRAM_CHANGE_REQUEST;
          SetCurrentProgram(ms70cdr.program - 1);
        }
        else if (button->pin == 3)
        {
          // new_task = PROGRAM_CHANGE_REQUEST;
          SetCurrentProgram(ms70cdr.program + 1);
        }
        else
        {
          SetCurrentEffect(button->pin - 4);
          ToggleEffectState(button->pin - 4);
          new_task = CURRENT_PATCH_SEND;

          // ToggleEffect(button->pin - 4);
          // new_task = CURRENT_PATCH_REQUEST;
        }
      }
      else
      {
        new_task = IDENTIFY_REQUEST;
      }
    }
  }
  if (DEBUG)
    printf("Button %d released\n", button->pin);
}

void GetCurrentEffect()
{
  ms70cdr.effect = (6 - ((((int)ms70cdr.patchData[130] & 1) << 2) + (((int)ms70cdr.patchData[125] & 8) >> 2) + (((int)ms70cdr.patchData[129] & 0x40) >> 6)));
  if (DEBUG)
    printf("Current Effect: %d\r\n", ms70cdr.effect);
}

uint8_t GetEffectState(uint8_t n)
{
  return ((((int)ms70cdr.patchData[effectState_idx[n]] & 1) << 6) != 0 ? 1 : 0);
}

void GetMaxEffects()
{
  ms70cdr.maxEffects = ((int)ms70cdr.patchData[130] & 0x1c) >> 2;
  if (DEBUG)
    printf("Max Effects: %d\r\n", ms70cdr.maxEffects);
}

void SetCurrentProgram(uint8_t program)
{
  if (program <= 1)
    program = 1;
  else if (program >= 49)
    program = 49;
  ms70cdr.program = program;
  send_program_change(program, 1);
}

void SetLEDStates()
{
  if ((int)ms70cdr.patchData[0] == 0xf0 && (int)ms70cdr.patchData[4] == 0x28)
  {
    for (uint8_t i = 0; i < 6; i++)
    {
      uint8_t state = 0;
      if (i < ms70cdr.maxEffects)
        state = GetEffectState(i);
      led_states[(i + 2)] = state;
    }
  }
}

void SetCurrentEffect(uint8_t n)
{
  n = 5 - n;
  ms70cdr.patchData[129] = ((int)ms70cdr.patchData[129] & ~0x40) + ((n & 1) << 6);
  ms70cdr.patchData[125] = ((int)ms70cdr.patchData[125] & ~0x8) + ((n & 2) << 2);
  ms70cdr.patchData[130] = ((int)ms70cdr.patchData[130] & ~1) + ((n & 4) >> 2);
}

void SetPatchEditable()
{
  print_buffer(syx_parameter_edit_enable, 6, "Trying to set patch editable");
  send_sysex(syx_parameter_edit_enable, 6);
}

void ToggleEffect(uint8_t n)
{
  uint8_t state = GetEffectState(n);
  syx_request_toggle_effect[5] = n;
  syx_request_toggle_effect[7] = (state == 1 ? 0 : 1);
  if (DEBUG)
    printf("Toggle Effect: %d, State: %d\r\n", n, (state == 1 ? 0 : 1));
  send_sysex(syx_request_toggle_effect, 10);
}

void ToggleEffectState(uint8_t n)
{
  if (DEBUG)
    printf("Effect: %d\r\n", n);
  if (n <= ms70cdr.maxEffects)
  {
    uint8_t state = GetEffectState(n);
    if (DEBUG)
      printf("Effect: %d State: %d, Pos: %d, Value:%02x, ", n, state, effectState_idx[n], ms70cdr.patchData[effectState_idx[n]]);
    ms70cdr.patchData[effectState_idx[n]] = (int)ms70cdr.patchData[effectState_idx[n]] + (state == 0 ? 1 : -1);
    if (DEBUG)
      printf("New Value: %02x\r\n", ms70cdr.patchData[effectState_idx[n]]);
  }
}

void print_buffer(uint8_t *data, uint8_t size, char *message)
{
  if (DEBUG)
  {
    printf(message);
    printf(" (%d): \r\n", size);
    for (int i = 0; i < size; i++)
    {
      if (i > 0 && i % 10 == 0)
        printf("\n");
      printf("%02x", data[i]);
      if ((i + 1) < size)
        printf(" ");
      else
        printf("\r\n");
    }
  }
}

// Core Tasks
void handle_states_and_tasks()
{
  if (core0_state == WAITING)
  {
    if (core0_task == NONE)
    {
      if (new_task == IDENTIFY_REQUEST)
      {
        if (DEBUG)
          printf("IDENTIFY_REQUEST\r\n");
        core0_state = BUSY;
        send_sysex(syx_indetity_request, 6);
        core0_task = IDENTIFY_REQUEST;
        new_task = NONE;
      }
      else if (new_task == EDITOR_ON_REQUEST)
      {
        if (DEBUG)
          printf("EDITOR_ON_REQUEST\r\n");
        core0_state = BUSY;
        send_sysex(syx_parameter_edit_enable, 6);
        core0_task = EDITOR_ON_REQUEST;
        new_task = NONE;
      }
      else if (new_task == CURRENT_PATCH_REQUEST)
      {
        if (DEBUG)
          printf("CURRENT_PATCH_REQUEST\r\n");
        core0_state = BUSY;
        send_sysex(syx_request_current_patch, 6);
        core0_task = CURRENT_PATCH_REQUEST;
        new_task = NONE;
      }
      else if (new_task == CURRENT_PATCH_SEND)
      {
        if (DEBUG)
          printf("CURRENT_PATCH_SEND\r\n");
        core0_state = BUSY;
        send_sysex(ms70cdr.patchData, 146);
        print_buffer(ms70cdr.patchData, 146, "Send Patch");
        core0_task = CURRENT_PATCH_SEND;
        new_task = NONE;
      }
      else if (new_task == TOGGLE_EFFECT_REQUEST)
      {
        if (DEBUG)
          printf("TOGGLE_EFFECT_REQUEST\r\n");
        core0_state = BUSY;
        send_sysex(syx_request_toggle_effect, 10);
        core0_task = TOGGLE_EFFECT_REQUEST;
        new_task = NONE;
      }
      else if (new_task == SET_PARAM_REQUEST)
      {
        if (DEBUG)
          printf("SET_PARAM_REQUEST\r\n");
        core0_state = BUSY;
        send_sysex(syx_parameter_edit, 10);
        core0_task = SET_PARAM_REQUEST;
        new_task = NONE;
      }
      else if (new_task == PROGRAM_CHANGE_REQUEST)
      {
        if (DEBUG)
          printf("PROGRAM_CHANGE_REQUEST\r\n");
        core0_state = BUSY;
        core0_task = PROGRAM_CHANGE_REQUEST;
        new_task = NONE;
      }
      else if (new_task == CURRENT_PROGRAM_REQUEST)
      {
        if (DEBUG)
          printf("CURRENT_PROGRAM_REQUEST\r\n");
        core0_state = BUSY;
        core0_task = CURRENT_PROGRAM_REQUEST;
        send_sysex(syx_request_current_program, 6);
        new_task = NONE;
      }
    }
  }
  else if (core0_state == BUSY)
  {
    if (core0_task == IDENTIFY_REQUEST)
    {
      if (sysex.status == COMPLETE)
      {
        core0_task = NONE;
        core0_state = WAITING;
        if (sysex.size == 15 && sysex.message[6] == 0x61)
        {
          memset(sysex.message, 0, 146);
          sysex.size = 0;
          sysex.status = EMPTY;
          if (DEBUG)
            printf("device identified\r\n");
          ms70cdr.status = IDENTIFIED;
          led_states[1] = HIGH;
          send_sysex(syx_parameter_edit_enable, 6);
          new_task = CURRENT_PROGRAM_REQUEST;
        }
      }
    }
    else if (core0_task == EDITOR_ON_REQUEST)
    {
      // No feedback from this task
      core0_task = NONE;
      core0_state = WAITING;
    }
    else if (core0_task == CURRENT_PROGRAM_REQUEST)
    {
      if (pc_message.status == COMPLETE)
      {
        if (DEBUG)
          printf("Program: %d, Channel: %d\r\n", pc_message.program, pc_message.channel);
        ms70cdr.program = pc_message.program;

        pc_message.program = 0;
        pc_message.channel = 0;
        pc_message.status = EMPTY;

        core0_task = NONE;
        core0_state = WAITING;
        new_task = CURRENT_PATCH_REQUEST;
      }
    }
    else if (core0_task == CURRENT_PATCH_REQUEST)
    {
      if (sysex.status == COMPLETE)
      {
        if (sysex.size == 146 && sysex.message[4] == 0x28)
        {
          memcpy(ms70cdr.patchData, sysex.message, 146);
          print_buffer(sysex.message, sysex.size, "Sysex patch data");
          memset(sysex.message, 0, 146);
          sysex.size = 0;
          sysex.status = EMPTY;

          GetMaxEffects();
          GetCurrentEffect();
          SetLEDStates();
          core0_task = NONE;
          core0_state = WAITING;
        }
      }
    }
    else if (core0_task == CURRENT_PATCH_SEND)
    {
      core0_task = NONE;
      core0_state = WAITING;
      new_task = CURRENT_PATCH_REQUEST;
    }
    else if (core0_task == TOGGLE_EFFECT_REQUEST)
    {
      if (sysex.status == COMPLETE)
      {
        core0_task = NONE;
        core0_state = WAITING;
      }
    }
    else if (core0_task == SET_PARAM_REQUEST)
    {
      if (sysex.status == COMPLETE)
      {
        core0_task = NONE;
        core0_state = WAITING;
      }
    }
    else if (core0_task == PROGRAM_CHANGE_REQUEST)
    {
      if (pc_message.status == COMPLETE)
      {
        if (DEBUG)
          printf("Program: %d, Channel: %d\r\n", pc_message.program, pc_message.channel);
        ms70cdr.program = pc_message.program;

        pc_message.program = 0;
        pc_message.channel = 0;
        pc_message.status = EMPTY;

        core0_task = NONE;
        core0_state = WAITING;
        new_task = CURRENT_PATCH_REQUEST;
      }
    }
    else if (core0_task == TOGGLE_EFFECT_STATE)
    {
      core0_task = NONE;
      core0_state = WAITING;
    }
  }
}

static void poll_midi_host_rx(void)
{
  if (!device_mounted)
  {
    if (!ms70cdr.addr)
      return;
    if (!tuh_midi_configured(ms70cdr.addr))
      return;
    if (!device_mounted)
      tuh_descriptor_get_device(ms70cdr.addr, &desc_device, 18, clone_descriptors, 0);
    if (tuh_midih_get_num_rx_cables(ms70cdr.addr) < 1)
      return;
  }
  else
    tuh_midi_read_poll(ms70cdr.addr);
}

static void midi_host_task(void)
{
  poll_midi_host_rx();
  // handle_core1_states_and_tasks();
  if (sysex_complete)
  {
    tuh_midi_stream_flush(ms70cdr.addr);
    sysex_complete = false;
  }
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
void tuh_midi_mount_cb(uint8_t daddr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
  ms70cdr.addr = daddr;
  ms70cdr.status = CONNECTED;
  if (DEBUG)
    printf("device connected\r\n");
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  ms70cdr.addr = 0;
  ms70cdr.status = DISCONNECTED;
  device_mounted = false;
  core0_state = IDLE;
  core0_task = NONE;
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    led_states[i] = LOW;
  }
  if (DEBUG)
    printf("device disconnected\r\n");
}

// Sends program change messages
void send_program_change(uint8_t program, uint8_t channel)
{
  if (core0_state == WAITING)
  {
    if (core0_task == NONE)
    {
      uint8_t data[2] = {(192 + channel), program};

      core0_state = BUSY;
      core0_task = PROGRAM_CHANGE_REQUEST;

      uint8_t packet[4] = {0x0c, (192 + channel), program, 0x00};
      tuh_midi_packet_write(ms70cdr.addr, packet);
      tuh_midi_stream_flush(ms70cdr.addr);

      core0_state = WAITING;
      core0_task = NONE;
      new_task = CURRENT_PROGRAM_REQUEST;
    }
  }
}

// Sends SysEx messages
void send_sysex(uint8_t *message, int size)
{
  int numpackets = ceil((double)size / 3.0);

  uint8_t packet[4];
  int index = 0;
  for (int i = 0; i < numpackets; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      // Add CIN
      if (j == 0)
      {
        if (i == numpackets - 1)
        {
          if (size - index == 1)
          {
            // SysEx ends on next byte
            packet[0] = 0x05;
          }
          else if (size - index == 2)
          {
            // SysEx ends on next 2 bytes
            packet[0] = 0x06;
          }
          else
          {
            // SysEx ends on next 3 bytes
            packet[0] = 0x07;
          }
        }
        else
        {
          // SysEx packet start
          packet[0] = 0x04;
        }
      }
      else
      {
        if (index < size)
        {
          // SysEx data
          packet[j] = message[index];
        }
        else
        {
          // Fill rest with 0
          packet[j] = 0x00;
        }
        index++;
      }
    }
    // Send 4 byte packets
    tuh_midi_packet_write(ms70cdr.addr, packet);
  }
  // Flag as ready to flush
  sysex_complete = true;
}

// Handles received SysEx message
void handle_sysex_rx_cb(uint8_t *packet, uint8_t cin)
{
  for (uint8_t i = 1; i < 4; i++)
  {
    if (packet[i] == 0xf0)
    {
      sysex.status = READING;
      sysex.size = 0;
      memset(sysex.message, 0, 146);
      sysex.message[sysex.size++] = packet[i];
    }
    else if (sysex.status == READING)
    {
      sysex.message[sysex.size++] = packet[i];
      if (packet[i] == 0xf7)
      {
        // print_buffer(sysex.message, sysex.size, "Received Sysex");
        sysex.status = COMPLETE;
        break;
      }
    }
  }
}

void handle_cc_rx_cb(uint8_t *packet)
{
  // if (DEBUG) printf("CC received\r\n");
  //  Do nothing
}

void handle_pc_rx_cb(uint8_t *packet)
{
  pc_message.program = packet[2];
  pc_message.channel = (packet[1] & 0x0f) + 1;
  pc_message.status = COMPLETE;
}

// Midi received callback
void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
  if (ms70cdr.addr == dev_addr)
  {
    int i = 1;
    while (num_packets > 0)
    {
      --num_packets;
      uint8_t packet[4];
      uint8_t cin;
      if (tuh_midi_packet_read(dev_addr, packet))
      {
        cin = packet[0];
        // SysEx message
        if (cin == 0x04 || cin == 0x05 || cin == 0x06 || cin == 0x07)
        {
          handle_sysex_rx_cb(packet, cin);
        }
        else if (cin == 0xb)
        {
          handle_cc_rx_cb(packet);
        }
        else if (cin == 0xc)
        {
          handle_pc_rx_cb(packet);
        }
      }
    }
  }
}

// Midi sent callback
void tuh_midi_tx_cb(uint8_t dev_addr)
{
  (void)dev_addr;
}

void clone_descriptors(tuh_xfer_t *xfer)
{
  if (XFER_RESULT_SUCCESS != xfer->result)
    return;
  uint8_t const daddr = xfer->daddr;
  device_mounted = true;
  ms70cdr.status = READY;
  core0_state = WAITING;
  led_states[0] = HIGH;
  new_task = IDENTIFY_REQUEST;
}