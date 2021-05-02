/**
 * @brief Button Change messages plugin
 * @file button_change.cpp
 * @author Ferdian Jovan <ferdian.jovan@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Ferdian Jovan.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ButtonChange.h>


namespace mavros {
namespace extra_plugins {

static constexpr double ACK_TIMEOUT_DEFAULT = 5.0;
using utils::enum_value;
using lock_guard = std::lock_guard<std::mutex>;
using unique_lock = std::unique_lock<std::mutex>;


class CommandTransaction {
public:
    std::mutex cond_mutex;
    std::condition_variable ack;
    uint16_t expected_command;
    uint8_t result;
    uint8_t sysid;
    uint8_t compid;

    explicit CommandTransaction(uint16_t command, uint8_t sysid, uint8_t compid) :
        ack(),
        sysid(sysid),
        compid(compid),
        expected_command(command),
        // Default result if wait ack timeout
        result(enum_value(mavlink::common::MAV_RESULT::FAILED))
    { }
};


class ButtonChangePlugin: public plugin::PluginBase {

public:
    ButtonChangePlugin(): PluginBase(), nh("~button_change") { }

    void initialize(UAS &uas_) override
    {
        PluginBase::initialize(uas_);

        double command_ack_timeout;
        nh.param("command_ack_timeout", command_ack_timeout, ACK_TIMEOUT_DEFAULT);
        command_ack_timeout_dt = ros::Duration(command_ack_timeout);

        button_change_srv = nh.advertiseService("send", &ButtonChangePlugin::bc_srv_cb, this);
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&ButtonChangePlugin::handle_command_ack)
        };
    }

private:
    using L_CommandTransaction = std::list<CommandTransaction>;
    L_CommandTransaction ack_waiting_list;
    std::mutex mutex;

    ros::NodeHandle nh;
    ros::ServiceServer button_change_srv;
    ros::Duration command_ack_timeout_dt;

    /* message handlers */
    void handle_command_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &ack)
    {
        lock_guard lock(mutex);

        for (auto &tr : ack_waiting_list) {
            if(tr.expected_command == ack.command && msg->compid==tr.compid && msg->sysid==tr.sysid) {
                tr.result = ack.result;
                tr.ack.notify_all();
                return;
            }
        }

        ROS_WARN_THROTTLE_NAMED(
            10, "cmd", "CMD: Unexpected command %u, result %u", ack.command, ack.result
        );
    }

    /* -*- mid-level functions -*- */
    bool wait_ack_for(CommandTransaction &tr) {
        unique_lock lock(tr.cond_mutex);
        if (tr.ack.wait_for(lock, std::chrono::nanoseconds(command_ack_timeout_dt.toNSec())) == std::cv_status::timeout) {
            ROS_WARN_NAMED("cmd", "CMD: Command %u -- wait ack timeout", tr.expected_command);
            return false;
        } else {
            return true;
        }
    }

    bool send_button_change_and_wait(
        int time_boot_ms, int last_change_ms, uint8_t command, uint8_t sysid, uint8_t compid,
        unsigned char &success, uint8_t &result) {
        using mavlink::common::MAV_RESULT;

        unique_lock lock(mutex);

        L_CommandTransaction::iterator ack_it;

        for (const auto &tr : ack_waiting_list) {
            if(tr.expected_command == (uint16_t)command) {
                ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Command %u already in progress", command);
                return false;
            }
        }


        bool is_ack_required = (m_uas->is_ardupilotmega() || m_uas->is_px4());
        if (is_ack_required)
            ack_it = ack_waiting_list.emplace(ack_waiting_list.end(), (uint16_t)command, sysid, compid);

        mavlink::common::msg::BUTTON_CHANGE button {};
        button.time_boot_ms = time_boot_ms;
        button.last_change_ms = last_change_ms;
        button.state = command;
        UAS_FCU(m_uas)->send_message_ignore_drop(button);

        if (is_ack_required) {
            lock.unlock();
            bool is_not_timeout = wait_ack_for(*ack_it);
            lock.lock();

            success = is_not_timeout && ack_it->result == enum_value(MAV_RESULT::ACCEPTED);
            result = ack_it->result;
            ack_waiting_list.erase(ack_it);
        }
        else {
            success = true;
            result = enum_value(MAV_RESULT::ACCEPTED);
        }

        return true;
    }

    /* -*- callbacks -*- */
    bool bc_srv_cb(mavros_msgs::ButtonChange::Request &req, mavros_msgs::ButtonChange::Response &res) {
        return send_button_change_and_wait(
                   req.time_boot_ms, req.last_change_ms, req.state, req.sysid, req.compid,
                   res.success, res.result
               );
    }

};
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ButtonChangePlugin, mavros::plugin::PluginBase)
