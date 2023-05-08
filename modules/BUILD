load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "object_fusion",
    srcs = [
        "object_fusion.cpp",
    ],
    hdrs = [
        "object_fusion.h",
    ],
    copts = ["-std=c++17"],
    features = [
        "treat_warnings_as_errors",
        "strict_clang_tidy_warnings",
        "strict_warnings",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/association:agent_associator",
        "//modules/fusion:agent_updater",
        "//modules/prediction:agent_predictor",
        "//modules/types:object_fusion_common",
    ],
)
