From ubuntu:18.04

RUN apt-get update \
	&& apt-get install -y \
	gcc-8 \
  g++-8 \
	cmake \
	git \
	wget \
	vim

RUN update-alternatives --install \
    /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave \
    /usr/bin/g++ g++ /usr/bin/g++-8 

# install fmt
RUN cd /tmp \
    && git clone https://github.com/fmtlib/fmt.git \
    && cd fmt \
    && mkdir _build; cd _build \
    && cmake .. \
    && make \
    && make install 


# install spdlog
RUN cd /tmp \
    && git clone https://github.com/gabime/spdlog.git \
    && cd spdlog \
    && mkdir _build; cd _build \
    && cmake .. \
    && make \
    && make install 



# install yaml-cpp
RUN cd /tmp \
    && git clone https://github.com/jbeder/yaml-cpp.git \
    && cd yaml-cpp \
    && mkdir _build; cd _build \
    && cmake .. \
    && make \
    && make install 


# install boost
RUN cd /tmp && wget http://downloads.sourceforge.net/project/boost/boost/1.72.0/boost_1_72_0.tar.gz \
    && tar xfz boost_1_72_0.tar.gz \
    && rm boost_1_72_0.tar.gz \
    && cd boost_1_72_0 \
    && ./bootstrap.sh --with-libraries=serialization \
    && ./b2 install 


COPY . /usr/src/cbag
WORKDIR /usr/src/cbag

# build cbag
RUN ./build_util.sh