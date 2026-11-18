#ifndef TRACKING_TEST_LOG_H
#define TRACKING_TEST_LOG_H
void tracking_test_log(const char *format, ...);
#define LOGERROR(...) tracking_test_log(__VA_ARGS__)
#define LOGWARNING(...) tracking_test_log(__VA_ARGS__)
#endif
