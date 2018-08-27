#include <boost/test/unit_test.hpp>
#include <shark_slam/iShark.hpp>

using namespace shark_slam;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_constructor_is_called)
{
    shark_slam::iShark slam;
}
