/*
 * Copyright (C) 2020, Andrey Stepanov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * \file
 * \author Andrey Stepanov
 * \copyright GNU General Public License v3.0
 */


#include "scale_panel.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <rviz/visualization_manager.h>
#include <OGRE/OgreSceneManager.h>
#include <QLocale>


namespace orbiterros {
namespace rviz_plugin {


constexpr char ScalePanel::panel_name[] = "scale_panel";
const QString ScalePanel::world_scale_property_name("World scale");


ScalePanel::ScalePanel(QWidget* parent):
        rviz::Panel(parent),
        world_scale_editor_(new QLineEdit),
        actual_scale_(1.0),
        scale_validator_(1e-15, 1e3, 16)
{
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(new QLabel(world_scale_property_name));
    world_scale_editor_->setText(QVariant(actual_scale_).toString());
    scale_validator_.setLocale(QLocale::English);
    scale_validator_.setNotation(QDoubleValidator::ScientificNotation);
    world_scale_editor_->setValidator(&scale_validator_);
    layout->addWidget(world_scale_editor_);
    setLayout(layout);
    bool success = connect(world_scale_editor_, SIGNAL(returnPressed()), this, SLOT(updateWorldScale()));
    Q_ASSERT(success);
}


void ScalePanel::updateWorldScale() {
    if (not world_scale_editor_->hasAcceptableInput()) {
        ROS_ERROR_NAMED(panel_name, "Invalid scale value (validator)");
        return;
    }

    bool ok;
    const float new_scale = QLocale(QLocale::English).toFloat(world_scale_editor_->text(), &ok);
    if (not ok) {
        ROS_ERROR_NAMED(panel_name, "Invalid scale value (conversion)");
        return;
    }

    setWorldScale(new_scale);
}


void ScalePanel::setWorldScale(const float& scale) {
    if (scale < 1e-15 or scale > 1e3) {
        ROS_ERROR_NAMED(panel_name, "Invalid scale value (value)");
        return;
    }

    this->vis_manager_->getSceneManager()->getRootSceneNode()->setScale(scale, scale, scale);
    ROS_INFO_NAMED(panel_name, "Set world scale to %.02e", scale);
    actual_scale_ = scale;

    Q_EMIT configChanged();
}


void ScalePanel::onInitialize() {
    Panel::onInitialize();
    updateWorldScale();
}


void ScalePanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
    config.mapSetValue(world_scale_property_name, actual_scale_);
    ROS_DEBUG_NAMED(panel_name, "Scale saved");
}


void ScalePanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
    QVariant scale;
    if (config.mapGetValue(world_scale_property_name, &scale)) {
        world_scale_editor_->setText(scale.toString());
        updateWorldScale();
    }
}


}
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orbiterros::rviz_plugin::ScalePanel, rviz::Panel)
