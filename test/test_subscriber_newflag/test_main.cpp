// Documents the rule: Simple_Subscriber::getItem() CLEARS isDataNew(). pio test -e native -f test_subscriber_newflag
#include <unity.h>

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

using namespace VCTR::Core;

void setUp() {}
void tearDown() {}

static void test_get_item_clears_new_flag() {
  Topic<int> t;
  Simple_Subscriber<int> s;
  s.subscribe(t);
  t.publish(1);
  TEST_ASSERT_TRUE(s.isDataNew());
  (void)s.getItem();
  TEST_ASSERT_FALSE(s.isDataNew()); // a second reader of the same subscriber steals the flag
}

static void test_dedicated_subscriber_keeps_flag() {
  Topic<int> t;
  Simple_Subscriber<int> checked, peek; // rule: extra readers get their own subscriber
  checked.subscribe(t);
  peek.subscribe(t);
  t.publish(1);
  (void)peek.getItem();
  TEST_ASSERT_TRUE(checked.isDataNew());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_get_item_clears_new_flag);
  RUN_TEST(test_dedicated_subscriber_keeps_flag);
  return UNITY_END();
}
