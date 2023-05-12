load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "gtest_unit_test",
    srcs = [
        "gtest_unit_test.cpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/google_test:gtest_main",
        "//third_party/google_test:gtest",
    ],
)
