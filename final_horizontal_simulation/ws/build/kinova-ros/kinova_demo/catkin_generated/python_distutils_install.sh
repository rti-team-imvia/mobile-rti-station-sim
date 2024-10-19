#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ram/ws/src/kinova-ros/kinova_demo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ram/ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ram/ws/install/lib/python2.7/dist-packages:/home/ram/ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ram/ws/build" \
    "/usr/bin/python2" \
    "/home/ram/ws/src/kinova-ros/kinova_demo/setup.py" \
    egg_info --egg-base /home/ram/ws/build/kinova-ros/kinova_demo \
    build --build-base "/home/ram/ws/build/kinova-ros/kinova_demo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ram/ws/install" --install-scripts="/home/ram/ws/install/bin"
