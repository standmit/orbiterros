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


#pragma once


#include <rviz/panel.h>


class QLineEdit;


namespace orbiterros {
namespace rviz_plugin {


class ScalePanel : public rviz::Panel {
    Q_OBJECT

    public:
        ScalePanel(QWidget* parent = nullptr);

        virtual void load(const rviz::Config& config);
        virtual void save(rviz::Config config);

        static const char panel_name[];

    public Q_SLOTS:
        void setWorldScale(const float& scale);

    protected Q_SLOTS:
        void updateWorldScale();

    protected:
        QLineEdit* world_scale_editor_;
        float actual_scale_;

        static const QString world_scale_property_name;
};


}
}
