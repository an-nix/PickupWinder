# Multi-stage Docker image for PickupWinder build environment.
# Stage 1 compiles the PRU toolchain and installs ARM cross-toolchain support.
# Stage 2 contains the final build image with the compiled toolchains copied in.

FROM debian:12-slim AS builder

ARG BUILDER_UID=1000
ARG BUILDER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive
ENV TOOLCHAIN_DIR=/root/x-tools
ENV PATH=${TOOLCHAIN_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf \
    automake \
  bc \
    bison \
    flex \
    gawk \
    build-essential \
    ccache \
    cpio \
    curl \
    diffstat \
    device-tree-compiler \
    g++ \
    gcc \
    git \
    help2man \
    libc6-dev \
    libexpat1-dev \
    libgmp-dev \
    libmpc-dev \
    libmpfr-dev \
    libncurses-dev \
    libtool \
    libtool-bin \
    make \
    perl \
    pkg-config \
    python3 \
    python3-pip \
    python3-setuptools \
    rsync \
    texinfo \
    unifdef \
    unzip \
    wget \
    xz-utils \
    zlib1g-dev \
    file \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/crosstool-ng/crosstool-ng.git /tmp/crosstool-ng \
    && cd /tmp/crosstool-ng \
    && ./bootstrap \
    && ./configure --prefix=/usr/local \
    && make -j"$(nproc)" \
    && make install \
    && rm -rf /tmp/crosstool-ng

RUN groupadd -g "${BUILDER_GID}" builder \
    && useradd -m -u "${BUILDER_UID}" -g "${BUILDER_GID}" builder \
    && mkdir -p /home/builder/toolchain-build /home/builder/x-tools /home/builder/toolchain-build\
    && chown -R builder:builder /home/builder/toolchain-build /home/builder/x-tools

WORKDIR /home/builder/toolchain-build

COPY docker/config /home/builder/toolchain-build/config

# Build PRU toolchain if a custom config is provided, otherwise build default.
RUN if [ -f /home/builder/toolchain-build/config/pru/.config ]; then \
      runuser -u builder -- bash -lc 'cp /home/builder/toolchain-build/config/pru/.config . && ct-ng build'; \
    else \
      runuser -u builder -- bash -lc 'ct-ng pru || true && if [ -f .config ]; then ct-ng build; else echo "WARNING: PRU toolchain config not generated"; fi'; \
    fi

# ARM toolchain build disabled for now (uncomment to re-enable).
# RUN if [ -f /home/builder/toolchain-build/config/arm/.config ]; then \
#       mkdir -p /home/builder/toolchain-build/arm && chown builder:builder /home/builder/toolchain-build/arm && cd /home/builder/toolchain-build/arm && \
#       runuser -u builder -- bash -lc 'cp /home/builder/toolchain-build/config/arm/.config . && ct-ng build'; \
#     else \
#       echo 'No custom ARM toolchain config provided; ARM cross-toolchain will not be built.'; \
#     fi

#FROM debian:12-slim AS runtime

#ENV DEBIAN_FRONTEND=noninteractive
#ENV TOOLCHAIN_DIR=/root/x-tools
#ENV PATH=${TOOLCHAIN_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

#RUN apt-get update && apt-get install -y --no-install-recommends \
#    build-essential \
#    gcc \
#    g++ \
#    make \
#    gawk \
#    python3 \
#    python3-pip \
#    device-tree-compiler \
#    file \
#""    && rm -rf /var/lib/apt/lists/*

#COPY --from=builder /home/builder/x-tools /root/x-tools
#COPY --from=builder /usr/local/bin/ct-ng /usr/local/bin/ct-ng
#COPY --from=builder /usr/local/share/crosstool-ng /usr/local/share/crosstool-ng

#WORKDIR /workspace
CMD ["bash"]
