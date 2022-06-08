FROM alpine:3.15.4

RUN apk add clang g++ boost-dev boost-static git make cmake musl gcc ccache
