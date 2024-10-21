#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER unc_ros_graph_generator

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "./unc_ros_graph_generator_tpp.h"

#if !defined(_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>

/*
 * Use LTTNG_UST_TRACEPOINT_EVENT(), LTTNG_UST_TRACEPOINT_EVENT_CLASS(),
 * LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(), and
 * LTTNG_UST_TRACEPOINT_LOGLEVEL() here.
 */

LTTNG_UST_TRACEPOINT_EVENT(
    /* Tracepoint provider name */
    unc_ros_graph_generator,

    /* Tracepoint name */
    callback_begin,

    /* Input arguments */
    LTTNG_UST_TP_ARGS(
        const char *, name,
        int, invoc_cnt
    ),

    /* Output event fields */
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_string(callback_name, name)
        lttng_ust_field_integer(int, invocation_count, invoc_cnt)
    )
)

LTTNG_UST_TRACEPOINT_EVENT(
    /* Tracepoint provider name */
    unc_ros_graph_generator,

    /* Tracepoint name */
    callback_end,

    /* Input arguments */
    LTTNG_UST_TP_ARGS(
        const char *, name,
        int, invoc_cnt
    ),

    /* Output event fields */
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_string(callback_name, name)
        lttng_ust_field_integer(int, invocation_count, invoc_cnt)
    )
)

LTTNG_UST_TRACEPOINT_EVENT(
    /* Tracepoint provider name */
    unc_ros_graph_generator,

    /* Tracepoint name */
    publish,

    /* Input arguments */
    LTTNG_UST_TP_ARGS(
        const char *, sender,
        const char *, topic,
        int, invoc_cnt
    ),

    /* Output event fields */
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_string(sender_callback_name, sender)
        lttng_ust_field_string(topic_name, topic)
        lttng_ust_field_integer(int, invocation_count, invoc_cnt)
    )
)

LTTNG_UST_TRACEPOINT_EVENT(
    /* Tracepoint provider name */
    unc_ros_graph_generator,

    /* Tracepoint name */
    subscribe,

    /* Input arguments */
    LTTNG_UST_TP_ARGS(
        const char *, subscriber,
        const char *, topic
    ),

    /* Output event fields */
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_string(subscriber_callback_name, subscriber)
        lttng_ust_field_string(topic_name, topic)
    )
)

#endif /* _TP_H */

#include <lttng/tracepoint-event.h>