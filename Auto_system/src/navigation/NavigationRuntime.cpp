#include "NavigationRuntime.h"

namespace RobotNavigation
{
Navigator& navigator()
{
    static Navigator instance;
    return instance;
}
} // namespace RobotNavigation
