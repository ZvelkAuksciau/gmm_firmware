#include <ch.h>
#include <hal.h>

#include <node.hpp>
#include <kmti/gimbal/MotorCommand.hpp>
#include <kmti/gimbal/MotorStatus.hpp>

#include <config/config.hpp>

#include <bootloader_interface/bootloader_interface.hpp>
#include "os.hpp"

#include <hardware.hpp>
#include "math.h"

#define M_PI 3.1415926f
#define M_2PI 2*M_PI


/*
 * standard 9600 baud serial config.
 */
static const SerialConfig serialCfg = {
    9600,
    0,
    0,
    0
};

#define BOARD_NORMAL                0x00
#define BOARD_CALIBRATING_NUM_POLES 0x01
#define BOARD_CALIBRATING_OFFSET    0x04

uint8_t g_boardStatus = 0;
float cmd_power = 0.0f;

float wrap_2PI(float radian)
{
    float res = fmodf(radian, M_2PI); //TODO: fix errno error
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

float wrap_PI(float radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

static THD_WORKING_AREA(waThread1, 128);
void Thread1(void) {
    chRegSetThreadName("blinker");

    while(1) {
        Hardware::setStatusLed(true);
        Hardware::setCANLed(false);
        chThdSleepMilliseconds(500);
        Hardware::setStatusLed(false);
        Hardware::setCANLed(true);
        chThdSleepMilliseconds(500);
    }
}

os::config::Param<uint8_t> num_poles("mot.num_poles", 7, 1, 255);
os::config::Param<float> enc_offset("mot.offset", 0.0f, -M_PI, M_PI); //Electrical and mechanical angle offset between electrical phase and encoder
os::config::Param<int8_t> direction("mot.dir", 1, -1, 1);
os::config::Param<uint8_t> axis_id("mot.axis_id", 0, 0, 2); //0 - Pitch; 1 - Roll; 2 - Yaw
os::config::Param<float> axis_offset("mot.enc_off", 0.0f, -M_PI, M_PI); //Axis offset used for figuring out gimbal frame position
os::config::Param<bool> calib_on_next_start("mot.calib", false);

float mot_pos_rad = 0.0f;

//Running at 5khz
static THD_WORKING_AREA(waRotoryEncThd, 256);
void RotoryEncThd(void) {
    chRegSetThreadName("rotary_position_sensor");

    const uint8_t AS5048A_ANGLE[2] = {0xFF, 0xFF};
    uint8_t spi_rx_buf[2];
    uint16_t mot_pos = 0;
    //float mot_pos_rad = 0.0f;

    enum {
        GO_TO_ZERO,
        READ_ZERO_POS,
        DO_ONE_ROTATION,
        DO_4_ROTATIONS,
        DO_4_REV_ROTATIONS,
    };
    uint8_t calibState = GO_TO_ZERO;

    int32_t avg_calc = 0;
    uint32_t avg_count = 0;
    float off_avg = 0.0f;
    float cmd_angle = 0.0f;
    bool dir_calibrated = false;

#define CALIBRATION_POWER 1.0f

    while(1) {
        spiAcquireBus(&SPID1);
        spiStart(&SPID1, &Hardware::spicfg);
        spiSelect(&SPID1);
        spiExchange(&SPID1, 2, AS5048A_ANGLE, &spi_rx_buf);
        spiUnselect(&SPID1);

        spiReleaseBus(&SPID1);
        mot_pos = spi_rx_buf[0] << 8 & 0x3F00;
        mot_pos |= spi_rx_buf[1] & 0x00FF;
        mot_pos_rad = mot_pos * 0.00038349519f;

        if(g_boardStatus == BOARD_NORMAL) {
            if(direction.get() != 0) {
                if(cmd_power > 0.0f) {
                    Hardware::enablePWMOutput();
                    float command = mot_pos_rad * num_poles.get() - enc_offset.get() + 1.570796f;
                    command *= direction.get();
                    Hardware::setPwmCommand(command, cmd_power);
                } else if(cmd_power < 0.0f) {
                    Hardware::enablePWMOutput();
                    float command = mot_pos_rad * num_poles.get() - enc_offset.get() - 1.570796f;
                    command *= direction.get();
                    Hardware::setPwmCommand(command, -cmd_power);
                } else {
                    Hardware::disablePWMOutput();
                }
            } else Hardware::disablePWMOutput();
        }else if(g_boardStatus & BOARD_CALIBRATING_NUM_POLES) {
            switch(calibState){
            case GO_TO_ZERO:
                cmd_angle = 0.0f;
                avg_count = 0;
                avg_calc = 0;
                Hardware::enablePWMOutput();
                Hardware::setPwmCommand(0.0f, CALIBRATION_POWER);
                chThdSleepMilliseconds(1000);
                calibState = READ_ZERO_POS;
                break;
            case READ_ZERO_POS:
                avg_calc += mot_pos;
                avg_count++;
                if(avg_count == 100) {
                    avg_calc /= avg_count;
                    calibState = DO_ONE_ROTATION;
                }
                break;
            case DO_ONE_ROTATION:
                cmd_angle += 0.005f;
                Hardware::setPwmCommand(cmd_angle, CALIBRATION_POWER);
                int32_t diff = mot_pos - avg_calc;
                if(cmd_angle > 1.0f && !dir_calibrated) {
                    if(diff > 0) {
                        //Node::publishKeyValue("mot_dir", 1.0f);
                        direction.set(1);
                        dir_calibrated = true;
                    } else if(diff < 0) {
                        //Node::publishKeyValue("mot_dir", -1.0f);
                        direction.set(-1);
                        dir_calibrated = true;
                    }
                }
                if(cmd_angle > 3.0f) {
                    if(diff < 100 && diff > -100) {
                        g_boardStatus &= ~BOARD_CALIBRATING_NUM_POLES;
                        calibState = GO_TO_ZERO;
                        Hardware::disablePWMOutput();
                        num_poles.set(round(cmd_angle/6.2831f));
                        //Node::publishKeyValue("mot_poles", num_poles.get());
                        os::config::save();
                    }
                }
                break;
            }
        } else if(g_boardStatus & BOARD_CALIBRATING_OFFSET) {
            switch(calibState){
            case GO_TO_ZERO:
                Hardware::enablePWMOutput();
                avg_count = 0;
                off_avg = 0.0f;
                Hardware::setPwmCommand(0.0f, CALIBRATION_POWER);
                chThdSleepMilliseconds(1000);
                cmd_angle = 0.0f;
                calibState = DO_4_ROTATIONS;
                break;
            case DO_4_ROTATIONS:
                off_avg += wrap_2PI(mot_pos_rad*num_poles.get() - cmd_angle);
                avg_count++;
                cmd_angle += direction.get() * 0.005f;
                Hardware::setPwmCommand(wrap_2PI(cmd_angle), CALIBRATION_POWER);
                if(fabs(cmd_angle) >= 4*M_2PI*num_poles.get()) {
                    off_avg /= avg_count;
                    //Node::publishKeyValue("mot_pos_off", off_avg);
                    enc_offset.set(off_avg);
                    off_avg = 0.0f;
                    avg_count = 0;
                    calibState = DO_4_REV_ROTATIONS;
                    cmd_angle = 0.0f;
                }
                break;
            case DO_4_REV_ROTATIONS:
                off_avg += wrap_2PI(mot_pos_rad*num_poles.get() - cmd_angle);
                avg_count++;
                cmd_angle -= direction.get() * 0.005f;
                Hardware::setPwmCommand(wrap_2PI(cmd_angle), CALIBRATION_POWER);
                if(fabs(cmd_angle) >= 4*M_2PI*num_poles.get()) {
                    off_avg /= avg_count;
                    //Node::publishKeyValue("mot_neg_off", off_avg);
                    enc_offset.set(enc_offset.get()/2.0f + off_avg/2.0f);
                    off_avg = 0.0f;
                    avg_count = 0;
                    calibState = GO_TO_ZERO;
                    g_boardStatus &= ~BOARD_CALIBRATING_OFFSET;
                    os::config::save();
                }
                break;
            }
        }
        chThdSleepMicroseconds(200);
    }
}

auto onFirmwareUpdateRequestedFromUAVCAN(
    const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request)
{
    /*
     * Checking preconditions
     */
    static bool already_in_progress = false;

    const std::uint8_t source_node_id =
        ((request.source_node_id > 0) &&
         (request.source_node_id <= uavcan::NodeID::Max) &&
         uavcan::NodeID(request.source_node_id).isUnicast()) ?
            request.source_node_id :
            request.getSrcNodeID().get();

//    os::lowsyslog("UAVCAN firmware update request from %d, source %d, path '%s'\n",
//                  request.getSrcNodeID().get(),
//                  source_node_id,
//                  request.image_file_remote_path.path.c_str());

    if (already_in_progress)
    {
//        os::lowsyslog("UAVCAN firmware update is already in progress, rejecting\n");
        return uavcan::protocol::file::BeginFirmwareUpdate::Response::ERROR_IN_PROGRESS;
    }

    /*
     * Initializing the app shared structure with proper arguments
     */
    //Node::Lock locker;

    bootloader_interface::AppShared shared;
    shared.can_bus_speed = Node::getCANBitRate();
    shared.uavcan_node_id = Node::getNode().getNodeID().get();
    shared.uavcan_fw_server_node_id = source_node_id;
    shared.stay_in_bootloader = true;

    std::strncpy(static_cast<char*>(&shared.uavcan_file_name[0]),       // This is really messy
                 request.image_file_remote_path.path.c_str(),
                 shared.UAVCANFileNameMaxLength);
    shared.uavcan_file_name[shared.UAVCANFileNameMaxLength - 1] = '\0';

    static_assert(request.image_file_remote_path.path.MaxSize < shared.UAVCANFileNameMaxLength, "Err...");

//    os::lowsyslog("Bootloader args: CAN bus bitrate: %u, local node ID: %d\n",
//                  unsigned(shared.can_bus_speed), shared.uavcan_node_id);

    /*
     * Commiting everything
     */
    bootloader_interface::writeSharedStruct(shared);

    NVIC_SystemReset();

    already_in_progress = true;

    os::lowsyslog("UAVCAN firmware update initiated\n");
    return uavcan::protocol::file::BeginFirmwareUpdate::Response::ERROR_OK;
}

systime_t lastCommandTime = 0;


int main(void) {
    //Hardware and chibios init
    os::watchdog::Timer wdt = Hardware::init();

    const auto fw_version = bootloader_interface::getFirmwareVersion();

    const auto app_shared_read_result =
            bootloader_interface::readAndInvalidateSharedStruct();
    const auto& app_shared = app_shared_read_result.first;
    const auto app_shared_available = app_shared_read_result.second;

    Node::init(1000000, 11, fw_version.major, fw_version.minor,
            fw_version.vcs_commit, fw_version.image_crc64we,
            &onFirmwareUpdateRequestedFromUAVCAN);

    if(calib_on_next_start.get()) {
        g_boardStatus |= BOARD_CALIBRATING_OFFSET | BOARD_CALIBRATING_NUM_POLES;
        calib_on_next_start.setAndSave(false);
    }
    chThdSleepMilliseconds(200);

    uavcan::Subscriber<kmti::gimbal::MotorCommand> mot_sub(Node::getNode());

    const int mot_sub_start_res =
            mot_sub.start(
                    [&](const uavcan::ReceivedDataStructure<kmti::gimbal::MotorCommand>& msg)
                    {
                        cmd_power = msg.cmd[axis_id.get()];
                        lastCommandTime = chVTGetSystemTime();
                    });

    uavcan::Publisher<kmti::gimbal::MotorStatus> mot_status(Node::getNode());
    mot_status.init();

    kmti::gimbal::MotorStatus mot_status_msg;

    if (mot_sub_start_res < 0) {
        chSysHalt("Failed to start subscriber");
    }

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO,
            (tfunc_t) Thread1, NULL);
    chThdCreateStatic(waRotoryEncThd, sizeof(waRotoryEncThd), NORMALPRIO + 10,
            (tfunc_t) RotoryEncThd, NULL);

    while (1) {
        if (lastCommandTime != 0
                && lastCommandTime + MS2ST(200) < chVTGetSystemTime()) {
            lastCommandTime = 0;
            Hardware::disablePWMOutput();
            cmd_power = 0.0f;
        }
        mot_status_msg.axis_id = axis_id.get();
        mot_status_msg.motor_pos_rad_raw = wrap_PI(mot_pos_rad);
        mot_status_msg.motor_pos_rad = wrap_PI(mot_pos_rad - axis_offset.get());
        mot_status.broadcast(mot_status_msg);
        chThdSleepMilliseconds(50);
        wdt.reset();
    }
}

