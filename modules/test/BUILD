load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "object_fusion_unit_test",
    timeout = "short",
    srcs = [
        "object_fusion_unit_test.cpp",
    ],
    copts = [
        "-std=c++17",
    ],
    features = [
        "treat_warnings_as_errors",
        "strict_warnings",
    ],
    visibility = [
        "//modules:__subpackages__",
    ],
    deps = [
        "//modules:object_fusion",
        "//third_party/google_test:gtest",
        "//third_party/google_test:gtest_main",
    ],
)
