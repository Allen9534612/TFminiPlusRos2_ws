#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>


class MultiSideAngleCalculator : public rclcpp::Node
{
public:

    using Range =
        sensor_msgs::msg::Range;


    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            Range,
            Range>;


    using Synchronizer =
        message_filters::Synchronizer<SyncPolicy>;


    MultiSideAngleCalculator()
        : Node("multi_angle_calculator"),

          /*
           * Distance between the two LiDARs.
           *
           * Unit: meter
           */

          L_right_(0.17),
          L_front_(0.175)
    {
        /*
         * ==============================
         * Subscribers
         * ==============================
         */


        sub_fl_.subscribe(
            this,
            "range/front_left");


        sub_fr_.subscribe(
            this,
            "range/front_right");


        sub_rf_.subscribe(
            this,
            "range/right_front");


        sub_rr_.subscribe(
            this,
            "range/right_rear");


        /*
         * ==============================
         * Right-side synchronization
         * ==============================
         */

        sync_right_ =
            std::make_shared<Synchronizer>(
                SyncPolicy(10),
                sub_rf_,
                sub_rr_);


        sync_right_->setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(0.1));


        sync_right_->registerCallback(
            std::bind(
                &MultiSideAngleCalculator::rightSideCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));


        /*
         * ==============================
         * Front-side synchronization
         * ==============================
         */

        sync_front_ =
            std::make_shared<Synchronizer>(
                SyncPolicy(10),
                sub_fl_,
                sub_fr_);


        sync_front_->setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(0.1));


        sync_front_->registerCallback(
            std::bind(
                &MultiSideAngleCalculator::frontSideCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));


        /*
         * ==============================
         * Timer
         * ==============================
         *
         * Print result every 200 ms.
         */

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(
                    &MultiSideAngleCalculator::timerCallback,
                    this));


        RCLCPP_INFO(
            this->get_logger(),
            "Multi-side angle calculator started.");
    }


private:

    /*
     * ==============================
     * Robot geometry
     * ==============================
     */

    const double L_right_;
    const double L_front_;


    /*
     * ==============================
     * Latest data
     * ==============================
     */

    double right_distance_ = 0.0;
    double right_angle_ = 0.0;

    double front_distance_ = 0.0;
    double front_angle_ = 0.0;


    std::mutex data_mutex_;


    /*
     * ==============================
     * Subscribers
     * ==============================
     */

    message_filters::Subscriber<Range>
        sub_fl_;

    message_filters::Subscriber<Range>
        sub_fr_;

    message_filters::Subscriber<Range>
        sub_rf_;

    message_filters::Subscriber<Range>
        sub_rr_;


    /*
     * ==============================
     * Synchronizers
     * ==============================
     */

    std::shared_ptr<Synchronizer>
        sync_right_;

    std::shared_ptr<Synchronizer>
        sync_front_;


    /*
     * ==============================
     * Timer
     * ==============================
     */

    rclcpp::TimerBase::SharedPtr timer_;


    /*
     * ==============================
     * Calculate angle
     * ==============================
     *
     * d1 = first LiDAR distance
     * d2 = second LiDAR distance
     * L  = distance between LiDARs
     *
     *
     *              LiDAR 1
     *                 ●
     *                 |
     *                 | d1
     *                 |
     *        robot    |
     * ----------------●
     *             \
     *              \
     *               ●
     *             LiDAR 2
     *
     *
     * angle = atan2(d1 - d2, L)
     */

    double calculateAngle(
        double d1,
        double d2,
        double L)
    {
        const double angle_rad =
            std::atan2(
                d1 - d2,
                L);


        const double angle_deg =
            angle_rad *
            180.0 /
            M_PI;


        return angle_deg;
    }


    /*
     * ==============================
     * Right LiDAR callback
     * ==============================
     */

    void rightSideCallback(
        const Range::ConstSharedPtr msg_rf,
        const Range::ConstSharedPtr msg_rr)
    {
        const double distance_rf =
            msg_rf->range;


        const double distance_rr =
            msg_rr->range;


        const double angle =
            calculateAngle(
                distance_rf,
                distance_rr,
                L_right_);


        const double average_distance =
            (distance_rf +
             distance_rr) /
            2.0;


        std::lock_guard<std::mutex> lock(
            data_mutex_);


        right_distance_ =
            average_distance;


        right_angle_ =
            angle;
    }


    /*
     * ==============================
     * Front LiDAR callback
     * ==============================
     */

    void frontSideCallback(
        const Range::ConstSharedPtr msg_fl,
        const Range::ConstSharedPtr msg_fr)
    {
        const double distance_fl =
            msg_fl->range;


        const double distance_fr =
            msg_fr->range;


        const double angle =
            calculateAngle(
                distance_fl,
                distance_fr,
                L_front_);


        const double average_distance =
            (distance_fl +
             distance_fr) /
            2.0;


        std::lock_guard<std::mutex> lock(
            data_mutex_);


        front_distance_ =
            average_distance;


        front_angle_ =
            angle;
    }


    /*
     * ==============================
     * Timer callback
     * ==============================
     */

    void timerCallback()
    {
        double right_distance;
        double right_angle;

        double front_distance;
        double front_angle;


        {
            std::lock_guard<std::mutex> lock(
                data_mutex_);


            right_distance =
                right_distance_;


            right_angle =
                right_angle_;


            front_distance =
                front_distance_;


            front_angle =
                front_angle_;
        }


        std::ostringstream output;


        output
            << std::fixed
            << std::setprecision(2)

            << "Right: "
            << right_distance
            << " m, "

            << std::setprecision(2)
            << right_angle
            << " deg"

            << " | "

            << "Front: "
            << front_distance
            << " m, "

            << std::setprecision(2)
            << front_angle
            << " deg";


        RCLCPP_INFO(
            this->get_logger(),
            "%s",
            output.str().c_str());
    }
};


int main(
    int argc,
    char *argv[])
{
    rclcpp::init(argc, argv);


    auto node =
        std::make_shared<
            MultiSideAngleCalculator>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}