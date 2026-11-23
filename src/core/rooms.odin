package voidengine

// ============================================================================
// Rooms and doors — the metroidvania room graph and fade transitions.
//
// A Room has world bounds (the camera clamps to them) and doors. A Door is a
// trigger rect that leads to a named door in another room. Room_Transition
// runs fade-out -> swap -> fade-in, and won't re-trigger until the player has
// stepped clear of every door (so arriving inside a door doesn't bounce you
// straight back).
//
//   if i, hit := room_transition_check(&rt, &rooms[cur], player_box); hit { ... }
//   if room_transition_update(&rt, dt) {            // true once, at full black
//       cur, _ = room_find(rooms, rt.target_room)
//       d, _ := room_door_find(&rooms[cur], rt.target_door)
//       spawn := door_spawn_point(rooms[cur].doors[d])
//   }
//   overlay_alpha := room_transition_alpha(&rt)
// ============================================================================

Door :: struct {
    name:         string,
    rect:         Rect2,
    target_room:  string,
    target_door:  string,
    spawn_offset: [2]f32, // where to put the player's feet relative to the door's bottom-center when arriving
}

Room :: struct {
    name:   string,
    bounds: Rect2,
    doors:  []Door,
}

Room_Transition_Phase :: enum u8 {
    Idle,
    Fade_Out,
    Fade_In,
}

Room_Transition :: struct {
    phase:       Room_Transition_Phase,
    t:           f32,
    fade_time:   f32, // seconds per half (default 0.25)
    target_room: string,
    target_door: string,
    armed:       bool, // false until the player is clear of all doors
}

room_find :: proc(rooms: []Room, name: string) -> (int, bool) {
    for r, i in rooms {
        if r.name == name { return i, true }
    }
    return -1, false
}

room_door_find :: proc(room: ^Room, name: string) -> (int, bool) {
    for d, i in room.doors {
        if d.name == name { return i, true }
    }
    return -1, false
}

room_door_at :: proc(room: ^Room, body: Rect2) -> (int, bool) {
    for d, i in room.doors {
        if rect2_overlaps(d.rect, body) { return i, true }
    }
    return -1, false
}

// Feet position for a player arriving through this door.
door_spawn_point :: proc "contextless" (d: Door) -> [2]f32 {
    return {d.rect.x + d.rect.w * 0.5 + d.spawn_offset.x, d.rect.y + d.rect.h + d.spawn_offset.y}
}

room_transition_busy :: proc "contextless" (rt: ^Room_Transition) -> bool {
    return rt.phase != .Idle
}

// Start a transition through door d. False if one is already running.
room_transition_start :: proc(rt: ^Room_Transition, d: Door) -> bool {
    if rt.phase != .Idle { return false }
    if rt.fade_time <= 0 { rt.fade_time = 0.25 }
    rt.phase = .Fade_Out
    rt.t = 0
    rt.target_room = d.target_room
    rt.target_door = d.target_door
    rt.armed = false
    return true
}

// Arms/disarms and starts a transition when the body enters a door.
room_transition_check :: proc(rt: ^Room_Transition, room: ^Room, body: Rect2) -> (door: int, started: bool) {
    if rt.phase != .Idle { return -1, false }
    i, hit := room_door_at(room, body)
    if !hit {
        rt.armed = true
        return -1, false
    }
    if !rt.armed { return i, false }
    return i, room_transition_start(rt, room.doors[i])
}

// Advance the fade. Returns true exactly once, at full black: swap rooms then.
room_transition_update :: proc "contextless" (rt: ^Room_Transition, dt: f32) -> (swap: bool) {
    switch rt.phase {
    case .Idle:
        return false
    case .Fade_Out:
        rt.t += dt
        if rt.t >= rt.fade_time {
            rt.phase = .Fade_In
            rt.t = 0
            return true
        }
    case .Fade_In:
        rt.t += dt
        if rt.t >= rt.fade_time {
            rt.phase = .Idle
            rt.t = 0
        }
    }
    return false
}

// Black overlay opacity, 0..1.
room_transition_alpha :: proc "contextless" (rt: ^Room_Transition) -> f32 {
    ft := rt.fade_time > 0 ? rt.fade_time : 0.25
    switch rt.phase {
    case .Fade_Out: return clampf(rt.t / ft, 0, 1)
    case .Fade_In:  return clampf(1 - rt.t / ft, 0, 1)
    case .Idle:     return 0
    }
    return 0
}
