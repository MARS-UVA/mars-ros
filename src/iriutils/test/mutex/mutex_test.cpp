// tests.cpp
#include "mutex.h"
#include <gtest/gtest.h>
 
TEST(mutexTest, try_enterSuccess) { 
    CMutex mutex;
    ASSERT_TRUE(mutex.try_enter());
    mutex.exit();
    ASSERT_TRUE(mutex.try_enter());
}
 
TEST(mutexTest, try_enterFailure) { 
    CMutex mutex;
    mutex.enter();
    ASSERT_FALSE(mutex.try_enter());
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
