load("@//third_party:remotes.bzl", "THIRD_PARTY_RESOURCES")
load("@//bazel/rules:artifactory.bzl", "artifactory_archive")

def load_google_test():
    if "google_test" not in native.existing_rules():
        artifactory_archive(
            name = "google_test",
            path = "googletest-release-1.11.0.tar.gz",
            repo = THIRD_PARTY_RESOURCES,
            sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
            strip_prefix = "googletest-release-1.11.0",
        )
