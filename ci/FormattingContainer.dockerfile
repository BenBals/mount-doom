FROM ubuntu:20.04

RUN apt-get update -qq && apt-get install -y -qq clang-format-11 git-core
