load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "tests",
    srcs = glob(["**/*.cc"]),
    deps = [
        "//src/lib:TrojanMap",
        "@googletest//:gtest_main",
    ],
    timeout="long",
)

cc_test(
    name = "trojanmap_test",
    srcs = ["trojanmap_test.cc"],
    deps = [
        "//src/lib:TrojanMap",
        "@googletest//:gtest_main",
    ],
    timeout="long",
)

cc_test(
    name = "trojanmap_test_student",
    srcs = ["trojanmap_test_student.cc"],
    deps = [
        "//src/lib:TrojanMap",
        "@googletest//:gtest_main",
    ],
    timeout="long",
)

cc_test(
    name = "trojanmap_test_grader",
    srcs = ["trojanmap_test_grader.cc"],
    deps = [
        "//src/lib:TrojanMap",
        "@googletest//:gtest_main",
    ],
)