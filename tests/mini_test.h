#ifndef MINI_TEST_H
#define MINI_TEST_H

// A deliberately tiny, header-only test framework — no external
// dependencies (no GTest/Catch2 download needed), just plain C++17.
// Enough structure to organize test cases and get a pass/fail summary,
// which is the point: showing the project is actually tested.

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <cmath>

namespace minitest {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, fn});
    }
};

inline int& failuresInCurrentTest() {
    static int count = 0;
    return count;
}

inline int runAll() {
    int passed = 0, failed = 0;
    for (auto& tc : registry()) {
        failuresInCurrentTest() = 0;
        std::cout << "  " << tc.name << " ... ";
        try {
            tc.fn();
        } catch (const std::exception& ex) {
            std::cout << "EXCEPTION: " << ex.what() << "\n";
            failed++;
            continue;
        }
        if (failuresInCurrentTest() == 0) {
            std::cout << "PASS\n";
            passed++;
        } else {
            failed++;
        }
    }
    std::cout << "\n" << passed << " passed, " << failed << " failed, "
              << (passed + failed) << " total\n";
    return failed == 0 ? 0 : 1;
}

} // namespace minitest

#define TEST_CASE(name) \
    void name(); \
    static minitest::Registrar registrar_##name(#name, name); \
    void name()

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            if (minitest::failuresInCurrentTest() == 0) std::cout << "FAIL\n"; \
            minitest::failuresInCurrentTest()++; \
            std::cout << "    check failed: " #cond \
                      << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

#define CHECK_NEAR(a, b, eps) \
    CHECK(std::fabs((a) - (b)) < (eps))

#endif // MINI_TEST_H
