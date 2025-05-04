#ifndef MISSION_FREEFALL_HPP
#define MISSION_FREEFALL_HPP


#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"

#include "telemetry.hpp"

#include "mission_abstract.hpp"

#include "../starship_flaps.hpp"


namespace VCTR
{


class MissionFreefall : public MissionAbstract, public Core::Task_Periodic
{
private:

    Core::Simple_Subscriber<Math::Vector<float, 4>> ctrlSubr_;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;

    CTRL::StarshipFlaps& flaps_;





};


}







#endif