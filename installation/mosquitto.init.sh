#! /bin/sh
set -e

# PIDFILE=/var/run/mosquitto.pid

# source function library
. /etc/init.d/functions

start() {
	echo "Starting Mosquitto"
	start-stop-daemon -S -x /usr/local/bin/mosquitto -- -c /usr/local/conf/mosquitto.roborio.conf --daemon
    echo "done."
}

stop() {
	echo "Stopping Mosquitto"
	start-stop-daemon -K -x "/usr/local/bin/mosquitto"
	echo "done."
}

case "$1" in
  start)
	start
	;;
  stop)
	stop
	;;

  reload|force-reload|restart)
	stop
	sleep 2
	start
	;;

  status)
	status /usr/local/bin/mosquitto
	exit $?
  ;;

  *)
	echo "Usage: /etc/init.d/mosquitto {start|stop|status|reload|force-reload|restart}"
	exit 1
esac

exit 0
