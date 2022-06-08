FROM alpine:3.15.4

RUN apk add clang g++ boost-dev git make cmake ccache
