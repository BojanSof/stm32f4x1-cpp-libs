#ifndef TOUCH_CALIBRATION_HPP
#define TOUCH_CALIBRATION_HPP

#include <array>
#include <numeric>
#include "Touch.hpp"

#include <EmbeddedGfx/Colors.hpp>
#include <EmbeddedGfx/Circle.hpp>

namespace Devices
{
  template <typename TouchT, typename DisplayT>
  class TouchCalibration
  {
    public:
      TouchCalibration(TouchT& touch, DisplayT& display)
        : touch_{touch}
        , display_{display}
      { }

      void calibrate()
      {
        // calibration points constants
        static constexpr uint8_t NumCalibrationPoints = 5;
        static constexpr size_t xCoordinates[] = {
                    display_.getWidth()/4
                  , 3*display_.getWidth()/4
                  , display_.getWidth()/2
                  , display_.getWidth()/4
                  , 3*display_.getWidth()/4
                };
        static constexpr size_t yCoordinates[] = {
                    display_.getHeight()/4
                  , 3*display_.getHeight()/4
                  , display_.getHeight()/2
                  , display_.getHeight()/4
                  , 3*display_.getHeight()/4
                };
        static constexpr float pointRadius = 20.0f;
        // arrays which hold the offsets
        std::array<int, NumCalibrationPoints> xOffsets{};
        std::array<int, NumCalibrationPoints> yOffsets{};

        auto& canvas = touch_.getCanvas();
        using CanvasT = std::remove_reference_t<decltype(canvas)>;
        for(size_t iPoint = 0; iPoint < NumCalibrationPoints; ++iPoint)
        {
          // clear the whole screen
          ///@todo change with filled rectangle
          canvas.clear(EmbeddedGfx::Colors::Black);
          // draw the point on the screen
          EmbeddedGfx::Circle point{xCoordinates[iPoint], yCoordinates[iPoint], pointRadius};
          canvas.draw(point);
          // wait for touch
          auto touchCoordinates = touch_.getCoordinates();
          while(!touchCoordinates.has_value())
          {
            touchCoordinates = touch_.getCoordinates();
          }
          // get the touch coordinates
          auto xTouch = touchCoordinates.value().x;
          auto yTouch = touchCoordinates.value().y;
          // calculate the offsets and store them in array
          xOffsets[iPoint] = xCoordinates[iPoint] - xTouch;
          yOffsets[iPoint] = yCoordinates[iPoint] - yTouch;
        }
        canvas.clear(EmbeddedGfx::Colors::Black);
        // calculate the average of the offsets
        auto xOffsetAvg = std::accumulate(xOffsets.cbegin(), xOffsets.cend(), 0) / NumCalibrationPoints;
        auto yOffsetAvg = std::accumulate(yOffsets.cbegin(), yOffsets.cend(), 0) / NumCalibrationPoints;
        // give the offsets to the touch
        // touch_.setCalibration(xOffsetAvg, yOffsetAvg);
      }
    private:
      TouchT& touch_;
      DisplayT& display_;
  };
}

#endif  // TOUCH_CALIBRATION_HPP