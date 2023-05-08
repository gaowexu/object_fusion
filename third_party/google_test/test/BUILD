load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "gtest_unit_test",
    srcs = [
        "gtest_unit_test.cpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
