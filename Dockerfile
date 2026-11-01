# Development image for fusion-engine.
# Later issues add GUI packages; keep this core toolchain minimal.
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        clang \
        cmake \
        curl \
        gcc \
        g++ \
        git \
        libeigen3-dev \
        libyaml-cpp-dev \
        ninja-build \
        python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
