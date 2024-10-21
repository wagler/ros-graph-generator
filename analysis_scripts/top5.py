import bt2
import sys
import collections


def top5proc():
    # Get the trace path from the first command-line argument
    it = bt2.TraceCollectionMessageIterator(sys.argv[1])

    # This counter dictionary will hold execution times:
    #
    #     Task command name -> Total execution time (ns)
    exec_times = collections.Counter()

    # This holds the last `sched_switch` timestamp
    last_ts = None

    for msg in it:
        # We only care about event messages
        if type(msg) is not bt2._EventMessageConst:
            continue

        # Event of the event message
        event = msg.event

        # Keep only `sched_switch` events
        if event.cls.name != 'sched_switch':
            continue

        # Keep only records of events which LTTng emitted from CPU 0
        if event.packet.context_field['cpu_id'] != 0:
            continue

        # Event timestamp (ns)
        cur_ts = msg.default_clock_snapshot.ns_from_origin

        if last_ts is None:
            # Start here
            last_ts = cur_ts

        # (Short) name of the previous task command
        prev_comm = str(event.payload_field['prev_comm'])

        # Initialize an entry in our dictionary if not done yet
        if prev_comm not in exec_times:
            exec_times[prev_comm] = 0

        # Compute previous command execution time
        diff = cur_ts - last_ts

        # Update execution time of this command
        exec_times[prev_comm] += diff

        # Update last timestamp
        last_ts = cur_ts

    # Print top 5
    for name, ns in exec_times.most_common(5):
        print('{:20}{} s'.format(name, ns / 1e9))


if __name__ == '__main__':
    top5proc()


    # RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 trace 
    # -k sched_switch sched_wakeup 
    # -u ros2* lttng_ust_cyg_profile*:func_* unc_ros_graph_generator* lttng_ust_statedump:* 
    # --list -s first_ros_graph_gen