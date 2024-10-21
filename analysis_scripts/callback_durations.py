import bt2
import sys
import collections
import code

invocations = {}

def extract():
    # Get the trace path from the first command-line argument
    it = bt2.TraceCollectionMessageIterator(sys.argv[1])

    for msg in it:
        # We only care about event messages
        if type(msg) is not bt2._EventMessageConst:
            continue

        # Event of the event message
        event = msg.event

        # Keep only `unc_ros_graph_generator` events
        if 'unc_ros_graph_generator:callback_' not in event.cls.name:
            continue

        invocation_count = str(event.payload_field['invocation_count'])

        # Check if the key exists; if not, create a new list
        if invocation_count not in invocations:
            invocations[invocation_count] = []

        # Insert the new element into the list
        invocations[invocation_count].append(msg)

    # local_scope = locals().copy()
    # local_scope['invocations'] = invocations  # Include the global variable

    # code.interact(local=local_scope)

    # for group in invocations.keys():
    #     print("--------------Invocation {}------------".format(group))
    #     print(len(invocations[group]))
    #     cb_set = set()
    #     for evt in invocations[group]:
    #         cb_set.add(evt['callback_name'])
    #     print(cb_set)

def print_callback_durations():

    # invocation group --> callback name --> start/end
    all_invocations = {}

    for invocation_group in invocations.keys():

        # Setting empty dicts for each invocation group
        all_invocations[invocation_group] = {}
        for msg in invocations[invocation_group]:
            event = msg.event
            if 'unc_ros_graph_generator:callback_begin' in event.cls.name:
                cb_name = str(event['callback_name'])
                all_invocations[invocation_group][cb_name] = { 
                    "start_time" : msg.default_clock_snapshot.ns_from_origin
                }
            elif 'unc_ros_graph_generator:callback_end' in event.cls.name:
                cb_name = str(event['callback_name'])
                assert(cb_name in all_invocations[invocation_group])
                all_invocations[invocation_group][cb_name]["end_time"] = msg.default_clock_snapshot.ns_from_origin
        
        #code.interact(local=locals())
    for cb in all_invocations['0']:
        tup = all_invocations['0'][cb]
        start = tup['start_time']
        end = tup['end_time']
        print("Callback \'{}\' [{}, {}], duration = {:.3f} us".format(cb, start, end, float(end-start)/1000))

def print_subscribers():

    it = bt2.TraceCollectionMessageIterator(sys.argv[1])
    for msg in it:
        # We only care about event messages
        if type(msg) is not bt2._EventMessageConst:
            continue

        # Event of the event message
        event = msg.event

        # Keep only `unc_ros_graph_generator` events
        if 'unc_ros_graph_generator:subscribe' not in event.cls.name:
            continue

        cb_name = str(event['subscriber_callback_name'])
        topic = str(event['topic_name'])

        print("Subscriber: Callback: {}, Topic: {}".format(cb_name, topic))


if __name__ == '__main__':
    extract()
    print_callback_durations()