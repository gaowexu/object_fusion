load("@//third_party/google_test:google_test.bzl", "load_google_test")
load("@//third_party/eigen:eigen.bzl", "load_eigen")

def load_third_party_deps():
    load_google_test()
    load_eigen()
