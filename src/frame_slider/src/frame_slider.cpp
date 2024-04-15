#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtk/gtk.h>

#include <Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <string>

using namespace std;

class QuickSlider {
protected:
    GtkWidget *_scale, *_label, *_valueLabel, *_box;
    double _value;

    void on_scale_value_changed() {
        GtkScale *scale = GTK_SCALE(_scale);
        _value = gtk_range_get_value(GTK_RANGE(scale));
        // cout << _value << endl;
        changeValueOnLabel();
    }
    
    void changeValueOnLabel() {
        std::stringstream out;
        out << std::fixed << std::setprecision(2) << _value;
        gtk_label_set_text(GTK_LABEL(_valueLabel), out.str().c_str());
    }

public:
    QuickSlider(string name, double minVal, double maxVal, double increment, double value) {
        _scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, -10, 10, 0.1);
        _label = gtk_label_new(name.c_str());
        _valueLabel = gtk_label_new("-0.00");
        _box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
        _value = value;
        gtk_range_set_value(GTK_RANGE(_scale), value);
        gtk_widget_set_size_request(_scale, 160, -1);
        gtk_scale_set_draw_value(GTK_SCALE(_scale), false);

        gtk_box_pack_start(GTK_BOX(_box), _label, TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_box), _scale, TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_box), _valueLabel, TRUE, TRUE, 0);

        // g_signal_connect(_scale, "value-changed", G_CALLBACK(&QuickSlider::on_scale_value_changed), this);
        g_signal_connect(_scale, "value-changed", G_CALLBACK(+[](GtkWidget *widget, gpointer data) {
            QuickSlider *myClass = static_cast<QuickSlider*>(data);
            myClass->on_scale_value_changed();
        }), this);


        changeValueOnLabel();
    }

    GtkWidget *getWidget() {
        return _box;
    }

    double getValue() {
        return _value;
    }
};

class MainWindow {
protected:
    GtkWidget *_window, *_vbox;
    QuickSlider _tX, _tY, _tZ, _rX, _rY, _rZ;

public:
    MainWindow() :
        _tX("TX", -10, 10, 0.1, 2.0),
        _tY("TY", -10, 10, 0.1, 0.0),
        _tZ("TZ", -10, 10, 0.1, 0.0),

        _rX("RX", -3.14, 3.14, 0.1, 0.0),
        _rY("RY", -3.14, 3.14, 0.1, 0.0),
        _rZ("RZ", -3.14, 3.14, 0.1, 0.0) {

        _window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(_window), "Frame Slider");
        // gtk_window_set_default_size(GTK_WINDOW(_window), 300, 200);
        g_signal_connect(_window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

        _vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_box_pack_start(GTK_BOX(_vbox), _tX.getWidget(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_vbox), _tY.getWidget(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_vbox), _tZ.getWidget(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_vbox), _rX.getWidget(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_vbox), _rY.getWidget(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(_vbox), _rZ.getWidget(), TRUE, TRUE, 0);
        
        gtk_container_add(GTK_CONTAINER(_window), _vbox);

        gtk_widget_show_all(_window);
    }

    double getTX() {
        return _tX.getValue();
    }

    double getTY() {
        return _tY.getValue();
    }

    double getTZ() {
        return _tZ.getValue();
    }

    double getRX() {
        return _rX.getValue();
    }

    double getRY() {
        return _rY.getValue();
    }

    double getRZ() {
        return _rZ.getValue();
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "frame_slider");
    gtk_init(&argc, &argv);
    
    MainWindow mainWindow;

    tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped outTF;

    outTF.header.frame_id = "fixed_link";
    outTF.child_frame_id = "base_link";

    while(ros::ok()) {
        outTF.header.stamp = ros::Time::now();

        outTF.transform.translation.x = mainWindow.getTX();
        outTF.transform.translation.y = mainWindow.getTY();
        outTF.transform.translation.z = mainWindow.getTZ();

        Eigen::AngleAxisd rollAngle(mainWindow.getRX(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(mainWindow.getRY(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(mainWindow.getRZ(), Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q(yawAngle * pitchAngle * rollAngle);

        outTF.transform.rotation.x = q.x();
        outTF.transform.rotation.y = q.y();
        outTF.transform.rotation.z = q.z();
        outTF.transform.rotation.w = q.w();

        br.sendTransform(outTF);
        gtk_main_iteration();
    }

    return 0;
}