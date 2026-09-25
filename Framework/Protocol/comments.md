
    Consistency checks - selectable, because whether either can fire is a property of the
    codec and the request table, not of the engine.

    Both run on a frame the parser has already resolved and IS_RX_VALID has already accepted,
    so neither is looking for line corruption. What each is actually looking for is stated
    below, in the terms that decide whether it is dead code for a given integration.

    ---------------------------------------------------------------------------
    PROTOCOL_CHECK_RX_FRAME_CONSISTENCY

    Asks whether FRAME_FORMAT - the shape the engine is about to take payload offsets from -
    is the shape the codec actually framed these bytes as. It is NOT a second opinion on the
    length: Packet_RxRemaining already held the capture to LENGTH_MAX, and the checksum has
    already covered exactly the bytes collected.

    DEAD when all of the following hold:

        1. PARSE_RX_LENGTH and PARSE_RX_FRAME derive the frame length and Meta.Length from the
           SAME wire field by the same arithmetic. Then one checksum-valid frame cannot yield
           two numbers, and the comparison is an identity.
        2. Every row's ID.FRAME_FORMAT names the shape the codec frames that id as. A codec
           that selects a shape by inspecting the id - a short sync layout and a long data
           layout, say - can disagree with a table that names the other one.
        3. No id reaches a handler under a row bound for a different id. It does whenever a
           chunk id with no row of its own arrives mid-exchange: Protocol_CaptureReqOfTable
           keeps the previous row standing, so the offsets come from that row's shape.

    LIVE when any of those fail - and (3) is the common one, because it is how a bulk transfer
    carries a stream through a single row. There the check is what stops a chunk framed one
    way being sliced by offsets belonging to another.

    A codec that derives Meta.Length from a field independent of the framing length - a
    payload count beside a frame count - makes this a genuine runtime guard on wire data, and
    it should stay on.

    ---------------------------------------------------------------------------
    PROTOCOL_CHECK_TX_FRAME_BOUNDS

    Asks whether the response the handler just staged fits the buffer it was built in. Meta
    on this side is the handler's, so this is a guard on application arithmetic, not on the
    wire.

    DEAD when every handler in the table sets Meta.Length to a compile-time constant - the
    sizeof of its response struct. That is most tables.

    LIVE when any handler computes it: a bulk transfer sizing a chunk from a cursor and a
    configured CHUNK_MAX, or a handler echoing a length the request asked for. A CHUNK_MAX
    set larger than PACKET_BUFFER_LENGTH minus the frame's overhead overruns on every chunk,
    and nothing else in the engine is positioned to notice.

    Keep it on for any socket carrying Protocol_DataMode or an equivalent.

#
