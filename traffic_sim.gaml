model SimpleCarNetwork

global {
    int nb_cars <- 10;
    int nb_autonomous_cars <- 10;
    
    graph road_network;
    float step <- 0.1;
    int street_length <- 40;
    
    // Define world bounds to show entire network (spawn nodes at -10 to 180)
    geometry shape <- envelope(rectangle({-20, -20}, {200, 200}));
    
    // Collision tracking
    int total_near_misses <- 0;
    int total_forced_brakes <- 0;
    int total_reactive_brakes <- 0;
    int total_opposing_lane_overtakes <- 0;
    
    // Throughput tracking
    int cars_completed <- 0;
    int avs_completed <- 0;
    float total_car_travel_time <- 0.0;
    float total_av_travel_time <- 0.0;
    
    // Mode: set by experiment
    bool spawn_regular_cars <- true;
    bool spawn_autonomous_cars <- true;
    
    init {
        // Create 5x5 grid of intersections
        loop i from: 0 to: 4 {
            loop j from: 0 to: 4 {
                create intersection {
                    location <- {i * street_length + 10, j * street_length + 10};
                    has_traffic_signal <- true;
                    time_to_change <- 8.0 + rnd(3.0);
                }
            }
        }
        
        // Synchronize traffic lights by row
        loop j from: 0 to: 4 {
            list<intersection> row_intersections <- intersection where (int((each.location.y - 10) / street_length) = j);
            string shared_state <- flip(0.5) ? "ns_green" : "ew_green";
            float shared_time <- 8.0 + rnd(3.0);
            loop inter over: row_intersections {
                inter.signal_state <- shared_state;
                inter.time_to_change <- shared_time;
                inter.counter <- rnd(shared_time);
            }
        }
        
        // Create spawn nodes around perimeter
        loop j from: 0 to: 4 {
            create spawn_node { location <- {j * street_length + 10, -10}; is_start <- true; }
            create spawn_node { location <- {j * street_length + 10, 4 * street_length + 20}; is_end <- true; }
        }
        loop i from: 1 to: 3 {
            create spawn_node { location <- {-10, i * street_length + 10}; is_start <- true; }
            create spawn_node { location <- {4 * street_length + 20, i * street_length + 10}; is_end <- true; }
        }
        
        // HORIZONTAL ROADS - More single-lane bidirectional for more dangerous overtaking
        loop i from: 0 to: 4 {
            loop j from: 0 to: 3 {
                intersection source <- intersection[i * 5 + j];
                intersection target <- intersection[i * 5 + j + 1];
                
                if (i = 2) {
                    // Center row: dual lane bidirectional
                    create road { source_node <- source; target_node <- target; shape <- line([source.location, target.location]); num_lanes <- 2; is_one_way <- false; direction <- "right"; }
                    create road { source_node <- target; target_node <- source; shape <- line([target.location, source.location]); num_lanes <- 2; is_one_way <- false; direction <- "left"; }
                } else {
                    // All other rows: single-lane bidirectional (allows dangerous overtaking)
                    create road { source_node <- source; target_node <- target; shape <- line([source.location, target.location]); num_lanes <- 1; is_one_way <- false; direction <- "right"; }
                    create road { source_node <- target; target_node <- source; shape <- line([target.location, source.location]); num_lanes <- 1; is_one_way <- false; direction <- "left"; }
                }
            }
        }
        
        // VERTICAL ROADS
        loop i from: 0 to: 3 {
            loop j from: 0 to: 4 {
                intersection source <- intersection[i * 5 + j];
                intersection target <- intersection[(i + 1) * 5 + j];
                
                if (j = 2) {
                    // Center column: dual lane bidirectional
                    create road { source_node <- source; target_node <- target; shape <- line([source.location, target.location]); num_lanes <- 2; is_one_way <- false; direction <- "down"; }
                    create road { source_node <- target; target_node <- source; shape <- line([target.location, source.location]); num_lanes <- 2; is_one_way <- false; direction <- "up"; }
                } else {
                    // All other columns: single-lane bidirectional
                    create road { source_node <- source; target_node <- target; shape <- line([source.location, target.location]); num_lanes <- 1; is_one_way <- false; direction <- "down"; }
                    create road { source_node <- target; target_node <- source; shape <- line([target.location, source.location]); num_lanes <- 1; is_one_way <- false; direction <- "up"; }
                }
            }
        }
        
        // Create entry/exit roads
        loop s over: spawn_node where each.is_start {
            intersection closest <- intersection closest_to s;
            create road { source_node <- s; target_node <- closest; shape <- line([s.location, closest.location]); num_lanes <- 1; is_one_way <- true; is_entry_exit <- true; direction <- "entry"; }
        }
        loop e over: spawn_node where each.is_end {
            intersection closest <- intersection closest_to e;
            create road { source_node <- closest; target_node <- e; shape <- line([closest.location, e.location]); num_lanes <- 1; is_one_way <- true; is_entry_exit <- true; direction <- "exit"; }
        }
        
        // Build road network
        road_network <- as_driving_graph(road, list<agent>(intersection) + list<agent>(spawn_node));
        
        // Create traffic lights
        loop r over: road where (!each.is_entry_exit and each.target_node is intersection) {
            intersection target <- intersection(r.target_node);
            if (target.has_traffic_signal) {
                r.linked_signal <- target;
                if (r.direction = "up" or r.direction = "down") { target.ns_roads << r; }
                else if (r.direction = "left" or r.direction = "right") { target.ew_roads << r; }
                
                point road_end <- last(r.shape.points);
                point road_start <- first(r.shape.points);
                float dx <- road_end.x - road_start.x;
                float dy <- road_end.y - road_start.y;
                float len <- sqrt(dx * dx + dy * dy);
                
                create traffic_light {
                    location <- {road_end.x - (dx / len) * 5.0, road_end.y - (dy / len) * 5.0};
                    stop_position <- {road_end.x - (dx / len) * 4.0, road_end.y - (dy / len) * 4.0};
                    my_intersection <- target;
                    my_road <- r;
                    light_direction <- r.direction;
                }
            }
        }
        
        // Spawn initial vehicles based on mode
        if (spawn_regular_cars) {
            loop i from: 1 to: nb_cars { create car { do initialize_trip; } }
        }
        if (spawn_autonomous_cars) {
            loop i from: 1 to: nb_autonomous_cars { create autonomous_car { do initialize_trip; } }
        }
    }
    
    reflex spawn_vehicles when: every(5 #cycles) {
        if (spawn_regular_cars and length(car) < nb_cars) {
            create car { do initialize_trip; }
        }
        if (spawn_autonomous_cars and length(autonomous_car) < nb_autonomous_cars) {
            create autonomous_car { do initialize_trip; }
        }
    }
    
    // Stop simulation after 5000 cycles
    reflex stop_simulation when: cycle >= 5000 {
        do pause;
    }
}

species spawn_node skills: [intersection_skill] {
    bool is_start <- false;
    bool is_end <- false;
    aspect default {
        draw square(3) color: is_start ? #lime : #orange border: is_start ? #darkgreen : #darkorange;
    }
}

species traffic_light {
    intersection my_intersection;
    road my_road;
    string light_direction;
    point stop_position;
    
    string get_current_color {
        if (my_intersection = nil) { return "green"; }
        bool is_ns <- (light_direction = "up" or light_direction = "down");
        if (is_ns) {
            if (my_intersection.signal_state = "ns_green") { return "green"; }
            if (my_intersection.signal_state = "ns_yellow") { return "yellow"; }
            return "red";
        } else {
            if (my_intersection.signal_state = "ew_green") { return "green"; }
            if (my_intersection.signal_state = "ew_yellow") { return "yellow"; }
            return "red";
        }
    }
    
    aspect default {
        string c <- get_current_color();
        draw circle(2.0) color: (c = "green") ? #green : ((c = "yellow") ? #yellow : #red) border: #black;
    }
}

species road skills: [road_skill] {
    int num_lanes <- 1;
    bool is_one_way <- false;
    bool is_entry_exit <- false;
    string direction <- "none";
    intersection linked_signal <- nil;
    int current_vehicle_count <- 0 update: length(base_vehicle where (each.get_current_road() = self));
    int capacity <- 3 update: num_lanes * 3;
    bool is_congested <- false update: current_vehicle_count >= capacity;
    
    aspect default {
        point start_pt <- first(shape.points);
        point end_pt <- last(shape.points);
        float dx <- end_pt.x - start_pt.x;
        float dy <- end_pt.y - start_pt.y;
        float len <- sqrt(dx * dx + dy * dy);
        
        float lane_width <- 2.0;
        float total_width <- num_lanes * lane_width;
        
        float offset <- 0.0;
        if (!is_one_way and !is_entry_exit) {
            offset <- total_width * 0.6;
        }
        
        float perp_x <- -dy / len * offset;
        float perp_y <- dx / len * offset;
        
        geometry offset_shape <- line([{start_pt.x + perp_x, start_pt.y + perp_y}, {end_pt.x + perp_x, end_pt.y + perp_y}]);
        
        rgb road_color <- is_congested ? rgb(180, 80, 80) : rgb(120, 120, 120);
        draw offset_shape + total_width color: road_color border: rgb(80, 80, 80);
        
        if (num_lanes = 2 and !is_entry_exit) { 
            draw offset_shape color: #white width: 0.3; 
        }
        
        if (is_one_way and !is_entry_exit) {
            float angle <- atan2(dy, dx) * 180 / #pi;
            int num_arrows <- max(1, int(len / 15));
            loop k from: 1 to: num_arrows {
                float t <- k / (num_arrows + 1);
                point arrow_pos <- {start_pt.x + dx * t + perp_x, start_pt.y + dy * t + perp_y};
                draw triangle(2.5) at: arrow_pos color: #white rotate: angle + 90;
            }
        }
        
        // Mark single-lane bidirectional roads (dangerous overtake zones)
        if (!is_one_way and num_lanes = 1 and !is_entry_exit) {
            float mid_t <- 0.5;
            point mid_pos <- {start_pt.x + dx * mid_t + perp_x, start_pt.y + dy * mid_t + perp_y};
            draw circle(1.5) at: mid_pos color: #yellow border: #orange;
        }
    }
}

species intersection skills: [intersection_skill] {
    bool has_traffic_signal <- false;
    string signal_state <- "ns_green";
    float time_to_change <- 8.0;
    float yellow_time <- 2.0;
    float counter <- 0.0;
    list<road> ns_roads <- [];
    list<road> ew_roads <- [];
    
    reflex change_signal when: has_traffic_signal {
        counter <- counter + step;
        float phase_duration <- (signal_state contains "yellow") ? yellow_time : time_to_change;
        if (counter >= phase_duration) {
            if (signal_state = "ns_green") { signal_state <- "ns_yellow"; }
            else if (signal_state = "ns_yellow") { signal_state <- "ew_green"; }
            else if (signal_state = "ew_green") { signal_state <- "ew_yellow"; }
            else { signal_state <- "ns_green"; }
            counter <- 0.0;
        }
    }
    
    // Calculate time until signal turns green for a given direction
    float time_to_green(string direction) {
        if (!has_traffic_signal) { return 0.0; }
        
        bool is_ns <- (direction = "up" or direction = "down");
        float remaining <- 0.0;
        
        if (is_ns) {
            if (signal_state = "ns_green") { return 0.0; }  // Already green
            else if (signal_state = "ns_yellow") {
                // Yellow -> EW green -> EW yellow -> NS green
                remaining <- (yellow_time - counter) + time_to_change + yellow_time;
            }
            else if (signal_state = "ew_green") {
                // EW green -> EW yellow -> NS green
                remaining <- (time_to_change - counter) + yellow_time;
            }
            else {  // ew_yellow
                remaining <- yellow_time - counter;  // Almost NS green
            }
        } else {
            if (signal_state = "ew_green") { return 0.0; }  // Already green
            else if (signal_state = "ew_yellow") {
                // Yellow -> NS green -> NS yellow -> EW green
                remaining <- (yellow_time - counter) + time_to_change + yellow_time;
            }
            else if (signal_state = "ns_green") {
                // NS green -> NS yellow -> EW green
                remaining <- (time_to_change - counter) + yellow_time;
            }
            else {  // ns_yellow
                remaining <- yellow_time - counter;  // Almost EW green
            }
        }
        return remaining;
    }
    
    // Get remaining green time for a direction (0 if red/yellow)
    float remaining_green_time(string direction) {
        if (!has_traffic_signal) { return 100.0; }  // No signal = always go
        
        bool is_ns <- (direction = "up" or direction = "down");
        
        if (is_ns and signal_state = "ns_green") {
            return time_to_change - counter;
        }
        else if (!is_ns and signal_state = "ew_green") {
            return time_to_change - counter;
        }
        return 0.0;  // Not currently green
    }
    
    // Check if direction is green
    bool is_green_for(string direction) {
        bool is_ns <- (direction = "up" or direction = "down");
        return (is_ns and signal_state = "ns_green") or (!is_ns and signal_state = "ew_green");
    }
    
    aspect default { draw square(6) color: rgb(100, 100, 100) border: rgb(60, 60, 60); }
}

species base_vehicle skills: [driving] {
    spawn_node start_node;
    spawn_node target_node;
    traffic_light nearest_light <- nil;
    float perception_distance <- 50.0;
    float min_safety_distance <- 4.0;
    float safety_distance_factor <- 0.5;
    bool is_stopped <- false;
    agent car_ahead <- nil;
    float distance_to_car_ahead <- 100.0;
    int current_lane <- 0;
    bool wants_to_overtake <- false;
    float collision_radius <- 1.5;
    bool emergency_brake <- false;
    list<base_vehicle> nearby_vehicles <- [];
    
    // Lane change animation
    int previous_lane <- 0;
    float lane_change_progress <- 1.0;
    float lane_change_speed <- 0.12;
    point lane_offset <- {0, 0};
    
    // Oncoming traffic detection
    base_vehicle oncoming_vehicle <- nil;
    float distance_to_oncoming <- 100.0;
    
    // Reactive braking
    bool reactive_brake <- false;
    base_vehicle threat_vehicle <- nil;
    float threat_distance <- 100.0;
    
    // Opposing lane overtake state
    bool in_opposing_lane <- false;
    float opposing_lane_timer <- 0.0;
    float max_opposing_time <- 20.0;  // Reduced for more urgency
    
    road get_current_road {
        return (current_road != nil) ? road(current_road) : nil;
    }
    
    float get_safety_distance { return min_safety_distance + (speed * safety_distance_factor); }

    point calculate_lane_offset {
        road my_road <- get_current_road();
        if (my_road = nil) { return {0, 0}; }
        
        point rd <- last(my_road.shape.points) - first(my_road.shape.points);
        float len <- sqrt(rd.x * rd.x + rd.y * rd.y);
        if (len < 0.001) { return {0, 0}; }
        
        float perp_x <- -rd.y / len;
        float perp_y <- rd.x / len;
        
        float base_offset <- 0.0;
        if (!my_road.is_one_way and !my_road.is_entry_exit) {
            base_offset <- my_road.num_lanes * 2.0 * 0.6;
        }
        
        float lane_off <- 0.0;
        if (my_road.num_lanes = 2) {
            float target_off <- (current_lane = 0) ? -1.0 : 1.0;
            if (lane_change_progress < 1.0) {
                float prev_off <- (previous_lane = 0) ? -1.0 : 1.0;
                lane_off <- prev_off + (target_off - prev_off) * lane_change_progress;
            } else {
                lane_off <- target_off;
            }
        }
        // Offset for opposing lane overtake on single-lane roads
        else if (in_opposing_lane and my_road.num_lanes = 1 and !my_road.is_one_way) {
            lane_off <- -4.0;  // Move to opposing lane side
        }
        
        return {perp_x * (base_offset + lane_off), perp_y * (base_offset + lane_off)};
    }
    
    point get_visual_position {
        return location + lane_offset;
    }
    
    action do_lane_change(int new_lane) {
        if (new_lane != current_lane and lane_change_progress >= 1.0) {
            previous_lane <- current_lane;
            current_lane <- new_lane;
            lane_change_progress <- 0.0;
        }
    }
    
    reflex animate_lane_change {
        if (lane_change_progress < 1.0) {
            lane_change_progress <- min(1.0, lane_change_progress + lane_change_speed);
        }
        lane_offset <- calculate_lane_offset();
    }
    
    action scan_nearby_vehicles {
        nearby_vehicles <- [];
        point my_pos <- get_visual_position();
        loop v over: base_vehicle {
            if (v != self) {
                point their_pos <- v.get_visual_position();
                if (my_pos distance_to their_pos < perception_distance) {
                    nearby_vehicles <- nearby_vehicles + v;
                }
            }
        }
    }
    
    action detect_incoming_threats {
        threat_vehicle <- nil;
        threat_distance <- 100.0;
        reactive_brake <- false;
        
        point my_pos <- get_visual_position();
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
        float road_len <- sqrt(road_dir.x * road_dir.x + road_dir.y * road_dir.y);
        if (road_len < 0.001) { return; }
        road_dir <- {road_dir.x / road_len, road_dir.y / road_len};
        
        loop v over: nearby_vehicles {
            point their_pos <- v.get_visual_position();
            float dist <- my_pos distance_to their_pos;
            
            // LOWERED threshold - detect threats at larger distance
            if (dist > min_safety_distance * 6) { continue; }
            
            bool changing_into_our_lane <- false;
            if (v.lane_change_progress < 1.0) {
                road v_road <- v.get_current_road();
                if (v_road = my_road) {
                    if (v.current_lane = current_lane and v.previous_lane != current_lane) {
                        changing_into_our_lane <- true;
                    }
                }
            }
            
            point to_them <- their_pos - my_pos;
            float forward_component <- to_them.x * road_dir.x + to_them.y * road_dir.y;
            float lateral_component <- abs(to_them.x * (-road_dir.y) + to_them.y * road_dir.x);
            
            bool cutting_in_front <- false;
            if (forward_component > 0 and forward_component < min_safety_distance * 3) {
                if (lateral_component < 5.0 and v.lane_change_progress < 0.9) {
                    cutting_in_front <- true;
                }
            }
            
            bool aggressive_approach <- false;
            if (forward_component < 0 and forward_component > -min_safety_distance * 3) {
                if (v.speed > speed + 1.5 and dist < min_safety_distance * 2) {
                    aggressive_approach <- true;
                }
            }
            
            // Detect vehicle in opposing lane coming at us
            bool opposing_lane_threat <- false;
            if (v.in_opposing_lane or in_opposing_lane) {
                if (dist < min_safety_distance * 4 and forward_component > 0) {
                    opposing_lane_threat <- true;
                }
            }
            
            bool dangerous_oncoming <- false;
            if (oncoming_vehicle = v and distance_to_oncoming < min_safety_distance * 3) {
                dangerous_oncoming <- true;
            }
            
            if ((changing_into_our_lane or cutting_in_front or dangerous_oncoming or opposing_lane_threat or aggressive_approach) and dist < threat_distance) {
                threat_vehicle <- v;
                threat_distance <- dist;
                
                // LOWERED threshold for reactive brake
                if (dist < min_safety_distance * 2.5) {
                    reactive_brake <- true;
                }
            }
        }
    }

    action find_oncoming_traffic {
        oncoming_vehicle <- nil;
        distance_to_oncoming <- 100.0;
        road my_road <- get_current_road();
        if (my_road = nil or my_road.is_one_way) { return; }
        
        point my_pos <- get_visual_position();
        point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
        float road_len <- sqrt(road_dir.x * road_dir.x + road_dir.y * road_dir.y);
        road_dir <- {road_dir.x / road_len, road_dir.y / road_len};
        
        loop v over: nearby_vehicles {
            float their_dir_x <- cos(v.heading * #pi / 180);
            float their_dir_y <- sin(v.heading * #pi / 180);
            float dot <- road_dir.x * their_dir_x + road_dir.y * their_dir_y;
            
            if (dot < -0.3) {  // LOWERED from -0.5 to detect more oncoming
                point their_pos <- v.get_visual_position();
                float dist <- my_pos distance_to their_pos;
                
                point to_them <- their_pos - my_pos;
                float forward_dot <- to_them.x * road_dir.x + to_them.y * road_dir.y;
                
                if (forward_dot > 0 and dist < distance_to_oncoming) {
                    oncoming_vehicle <- v;
                    distance_to_oncoming <- dist;
                }
            }
        }
    }
    
    bool check_collision_risk {
        point my_pos <- get_visual_position();
        loop v over: nearby_vehicles {
            point their_pos <- v.get_visual_position();
            float dist <- my_pos distance_to their_pos;
            if (dist < collision_radius + v.collision_radius + 1.5) { return true; }
            if (speed > 0.1) {
                point my_future <- my_pos + {cos(heading) * speed * 2, sin(heading) * speed * 2};
                point their_future <- their_pos + {cos(v.heading) * v.speed * 2, sin(v.heading) * v.speed * 2};
                if (my_future distance_to their_future < collision_radius + v.collision_radius + 2.5) { return true; }
            }
        }
        return false;
    }
    
    action find_car_ahead {
        car_ahead <- nil;
        distance_to_car_ahead <- 100.0;
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        point my_pos <- get_visual_position();
        
        loop v over: nearby_vehicles {
            road v_road <- v.get_current_road();
            if (v_road = my_road) {
                // Skip vehicles in opposing lane when checking car ahead
                if (v.in_opposing_lane != in_opposing_lane) { continue; }
                
                if (my_road.num_lanes = 2) {
                    bool same_lane <- (v.current_lane = current_lane) or
                                      (v.lane_change_progress < 0.5 and v.previous_lane = current_lane) or
                                      (lane_change_progress < 0.5 and previous_lane = v.current_lane);
                    if (!same_lane) { continue; }
                }
                
                point their_pos <- v.get_visual_position();
                float dist <- my_pos distance_to their_pos;
                point to_other <- their_pos - my_pos;
                point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
                if (to_other.x * road_dir.x + to_other.y * road_dir.y > 0 and dist < distance_to_car_ahead) {
                    car_ahead <- v;
                    distance_to_car_ahead <- dist;
                }
            }
        }
    }
    
    bool is_lane_clear(int target_lane) {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes < 2) { return false; }
        
        point my_pos <- get_visual_position();
        
        loop v over: nearby_vehicles {
            road v_road <- v.get_current_road();
            if (v_road = my_road) {
                bool in_target <- (v.current_lane = target_lane) or
                                  (v.lane_change_progress < 1.0 and v.current_lane = target_lane);
                if (in_target) {
                    point their_pos <- v.get_visual_position();
                    if (my_pos distance_to their_pos < min_safety_distance * 2.5) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    // Check if opposing lane is clear enough for overtake
    bool is_opposing_lane_clear(float required_distance) {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.is_one_way) { return false; }
        
        // Check for oncoming traffic
        if (oncoming_vehicle != nil and distance_to_oncoming < required_distance) {
            return false;
        }
        
        // Check for other vehicles already in opposing lane
        point my_pos <- get_visual_position();
        loop v over: nearby_vehicles {
            if (v.in_opposing_lane) {
                point their_pos <- v.get_visual_position();
                if (my_pos distance_to their_pos < required_distance * 0.6) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    action find_traffic_light {
        nearest_light <- nil;
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        float min_dist <- perception_distance + 1.0;
        loop tl over: traffic_light where (each.my_road = my_road) {
            float dist <- location distance_to tl.location;
            if (dist <= perception_distance) {
                point to_light <- tl.location - location;
                point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
                if (to_light.x * road_dir.x + to_light.y * road_dir.y > 0 and dist < min_dist) {
                    nearest_light <- tl;
                    min_dist <- dist;
                }
            }
        }
    }
}

species car parent: base_vehicle {
    rgb color <- #blue;
    float max_speed <- 3.0 + rnd(5.0);  // HIGH variance: 3-8 speed
    float aggressiveness <- rnd(1.0);
    bool is_tailgating <- false;
    bool is_cutting_off <- false;
    bool honking <- false;
    int patience <- 0;
    int max_patience <- 15;  // Very impatient
    float tailgate_timer <- 0.0;
    float spawn_time <- 0.0;  // Track when spawned for throughput
    
    // CHAOTIC HUMAN BEHAVIOR
    float attention_level <- 0.5 + rnd(0.5);  // 0.5-1.0 how attentive
    float reaction_delay <- rnd(0.3);  // Delayed reactions
    bool is_distracted <- false;
    float distraction_timer <- 0.0;
    float speed_variance <- rnd(0.4);  // Random speed fluctuations
    bool is_aggressive_driver <- false;  // Set in init based on aggressiveness
    bool is_timid_driver <- false;
    float personal_space <- 1.0 + rnd(3.0);  // Variable following preference
    
    // REACTION TO BEING TAILGATED/HONKED AT
    bool being_tailgated <- false;
    bool being_honked_at <- false;
    float stress_level <- 0.0;  // Builds up when tailgated/honked at
    bool is_brake_checking <- false;  // Aggressive response - tap brakes
    bool is_nervous <- false;  // Timid response - erratic driving
    float road_rage_timer <- 0.0;  // How long they've been aggravated
    
    action initialize_trip {
        start_node <- one_of(spawn_node where each.is_start);
        target_node <- one_of(spawn_node where each.is_end);
        location <- start_node.location;
        spawn_time <- time;  // Record spawn time
        
        // DISTINCT driver personalities
        is_aggressive_driver <- aggressiveness > 0.7;
        is_timid_driver <- aggressiveness < 0.3;
        
        // Aggressive drivers: close following, fast lane changes, low patience
        if (is_aggressive_driver) {
            min_safety_distance <- 1.0 + rnd(1.0);  // Dangerously close
            safety_distance_factor <- 0.1 + rnd(0.1);
            max_patience <- int(5 + rnd(10));
            lane_change_speed <- 0.2 + rnd(0.1);  // Fast, jerky
            max_speed <- max_speed * 1.2;  // Speeders
        }
        // Timid drivers: far following, slow lane changes, high patience but nervous
        else if (is_timid_driver) {
            min_safety_distance <- 5.0 + rnd(3.0);  // Very far back
            safety_distance_factor <- 0.8 + rnd(0.3);
            max_patience <- int(80 + rnd(40));
            lane_change_speed <- 0.05 + rnd(0.03);  // Slow, hesitant
            max_speed <- max_speed * 0.7;  // Slow drivers
        }
        // Normal drivers: variable behavior
        else {
            min_safety_distance <- 2.0 + rnd(2.0) + (1.0 - aggressiveness) * 2.0;
            safety_distance_factor <- 0.3 + rnd(0.3);
            max_patience <- int(15 + rnd(30));
            lane_change_speed <- 0.08 + rnd(0.08);
        }
        
        current_lane <- 0;
        do compute_path(graph: road_network, source: start_node, target: target_node);
    }
    
    reflex perceive {
        // HUMAN DISTRACTION - sometimes miss things
        if (flip(0.02)) {  // 2% chance to become distracted each step
            is_distracted <- true;
            distraction_timer <- 0.0;
        }
        if (is_distracted) {
            distraction_timer <- distraction_timer + step;
            if (distraction_timer > 1.0 + rnd(2.0)) {
                is_distracted <- false;
            }
        }
        
        // Distracted drivers perceive less
        do scan_nearby_vehicles;
        if (!is_distracted or flip(attention_level)) {
            do find_traffic_light;
        }
        do find_car_ahead;
        if (!is_distracted or flip(attention_level * 0.8)) {
            do find_oncoming_traffic;
            do detect_incoming_threats;
        }
    }
    
    reflex manage_opposing_lane_timer when: in_opposing_lane {
        opposing_lane_timer <- opposing_lane_timer + step;
        
        // Abort if taking too long or oncoming traffic approaching
        if (opposing_lane_timer > max_opposing_time or 
            (oncoming_vehicle != nil and distance_to_oncoming < 25.0)) {
            // Try to return to own lane
            in_opposing_lane <- false;
            opposing_lane_timer <- 0.0;
            wants_to_overtake <- false;
            
            if (oncoming_vehicle != nil and distance_to_oncoming < 18.0) {
                total_near_misses <- total_near_misses + 1;
                total_reactive_brakes <- total_reactive_brakes + 1;
                speed <- speed * 0.3;
            }
        }
    }
    
    reflex emergency_collision_avoidance {
        emergency_brake <- false;
        point my_pos <- get_visual_position();
        loop v over: nearby_vehicles {
            point their_pos <- v.get_visual_position();
            float dist <- my_pos distance_to their_pos;
            float combined <- collision_radius + v.collision_radius;
            if (dist < combined + 2.5) {
                emergency_brake <- true; speed <- 0.0; real_speed <- 0.0; is_stopped <- true;
                total_forced_brakes <- total_forced_brakes + 1;
                // Abort opposing lane maneuver
                if (in_opposing_lane) {
                    in_opposing_lane <- false;
                    opposing_lane_timer <- 0.0;
                    total_near_misses <- total_near_misses + 1;
                }
                return;
            }
            if (dist < combined + 5.0 and speed > 0.5) {
                total_near_misses <- total_near_misses + 1;
                total_reactive_brakes <- total_reactive_brakes + 1;
                speed <- speed * 0.3;
            }
        }
        if (check_collision_risk()) { 
            speed <- max(0.0, speed * 0.5); 
            total_reactive_brakes <- total_reactive_brakes + 1;
        }
    }
    
    reflex react_to_threat when: threat_vehicle != nil {
        if (reactive_brake) {
            total_near_misses <- total_near_misses + 1;
            total_reactive_brakes <- total_reactive_brakes + 1;
            
            if (threat_distance < collision_radius * 4) {
                total_forced_brakes <- total_forced_brakes + 1;
                speed <- 0.0;
                is_stopped <- true;
                emergency_brake <- true;
                // Abort opposing lane
                if (in_opposing_lane) {
                    in_opposing_lane <- false;
                    opposing_lane_timer <- 0.0;
                }
            } else if (threat_distance < min_safety_distance * 1.5) {
                speed <- speed * 0.3;
                total_forced_brakes <- total_forced_brakes + 1;
            } else {
                speed <- speed * 0.6;
            }
        } else if (threat_distance < min_safety_distance * 2.5) {
            speed <- speed * 0.8;
        }
    }
    
    reflex react_to_oncoming when: oncoming_vehicle != nil and !in_opposing_lane {
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        if (!my_road.is_one_way and my_road.num_lanes = 1) {
            if (distance_to_oncoming < 18.0) {
                if (distance_to_oncoming < 10.0) {
                    total_near_misses <- total_near_misses + 1;
                    total_reactive_brakes <- total_reactive_brakes + 1;
                    speed <- max(0.0, speed * 0.2);
                    is_stopped <- true;
                } else {
                    total_reactive_brakes <- total_reactive_brakes + 1;
                    speed <- max(0.5, speed * 0.5);
                }
            } else if (distance_to_oncoming < 30.0) {
                speed <- min(speed, max_speed * 0.5);
            }
        }
    }
    
    // React when someone is overtaking in opposing lane - yield to let them back in
    reflex yield_to_opposing_overtaker {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes != 1 or my_road.is_one_way) { return; }
        
        point my_pos <- get_visual_position();
        
        loop v over: nearby_vehicles {
            if (v.in_opposing_lane) {
                road v_road <- v.get_current_road();
                // Check if they're on same road segment or parallel
                if (v_road = my_road or (v_road != nil and v_road.source_node = my_road.target_node)) {
                    point their_pos <- v.get_visual_position();
                    float dist <- my_pos distance_to their_pos;
                    
                    // They're trying to merge back - slow down to create gap
                    if (dist < min_safety_distance * 5) {
                        point to_them <- their_pos - my_pos;
                        point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
                        float road_len <- sqrt(road_dir.x * road_dir.x + road_dir.y * road_dir.y);
                        road_dir <- {road_dir.x / road_len, road_dir.y / road_len};
                        
                        float forward_dot <- to_them.x * road_dir.x + to_them.y * road_dir.y;
                        
                        // They're ahead or beside us - create space
                        if (forward_dot > -min_safety_distance) {
                            if (aggressiveness < 0.6) {
                                // Non-aggressive drivers yield
                                speed <- speed * 0.6;
                                total_reactive_brakes <- total_reactive_brakes + 1;
                            } else if (aggressiveness > 0.75 and dist < min_safety_distance * 2.5) {
                                // Very aggressive - don't yield, force them to abort
                                speed <- min(speed * 1.1, max_speed);
                                honking <- true;
                                total_near_misses <- total_near_misses + 1;
                            }
                        }
                    }
                }
            }
        }
    }
    
    reflex detect_tailgating {
        is_tailgating <- false;
        honking <- false;
        
        if (car_ahead != nil) {
            float safe_dist <- get_safety_distance();
            base_vehicle ahead <- base_vehicle(car_ahead);
            
            // AGGRESSIVE DRIVERS tailgate constantly
            if (is_aggressive_driver) {
                if (distance_to_car_ahead < safe_dist * 3.0 and distance_to_car_ahead > collision_radius * 2) {
                    if (ahead != nil and ahead.speed < max_speed * 0.9) {
                        is_tailgating <- true;
                        tailgate_timer <- tailgate_timer + step;
                        
                        // Aggressive: honk quickly and often
                        if (tailgate_timer > 0.5) {
                            honking <- true;
                            patience <- patience + 4;  // Very impatient
                        }
                        
                        // Flash lights (visual only) and get VERY close
                        if (patience > max_patience * 0.3) {
                            min_safety_distance <- max(0.8, min_safety_distance - 0.1);
                        }
                    }
                }
            }
            // Normal drivers sometimes tailgate
            else if (!is_timid_driver and aggressiveness > 0.3) {
                if (distance_to_car_ahead < safe_dist * 2.0 and distance_to_car_ahead > collision_radius * 2) {
                    if (ahead != nil and ahead.speed < max_speed * 0.8) {
                        is_tailgating <- true;
                        tailgate_timer <- tailgate_timer + step;
                        
                        if (tailgate_timer > 2.0 or aggressiveness > 0.5) {
                            honking <- flip(0.3);  // Sometimes honk
                            patience <- patience + 2;
                        }
                    }
                }
            }
            // Timid drivers NEVER tailgate but get nervous when followed closely
            else if (is_timid_driver) {
                tailgate_timer <- 0.0;
            }
            
            if (!is_tailgating) {
                tailgate_timer <- 0.0;
            }
        }
    }
    
    // Detect if someone is tailgating/honking at US from behind
    reflex detect_being_tailgated {
        being_tailgated <- false;
        being_honked_at <- false;
        
        point my_pos <- get_visual_position();
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        // Check for vehicles behind us
        loop v over: nearby_vehicles {
            if (species(v) = car) {
                car c <- car(v);
                point their_pos <- c.get_visual_position();
                float dist <- my_pos distance_to their_pos;
                
                // Check if they're behind us on same road
                if (dist < 20.0) {
                    point to_them <- their_pos - my_pos;
                    point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
                    float road_len <- sqrt(road_dir.x * road_dir.x + road_dir.y * road_dir.y);
                    if (road_len > 0) {
                        road_dir <- {road_dir.x / road_len, road_dir.y / road_len};
                        float behind_dot <- to_them.x * road_dir.x + to_them.y * road_dir.y;
                        
                        // They're behind us (negative dot product)
                        if (behind_dot < -1.0 and dist < 12.0) {
                            if (c.is_tailgating) {
                                being_tailgated <- true;
                            }
                            if (c.honking) {
                                being_honked_at <- true;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // REACT to being tailgated or honked at - various human responses
    reflex react_to_tailgating when: being_tailgated or being_honked_at {
        // Build up stress
        float stress_increase <- 0.0;
        if (being_tailgated) { stress_increase <- stress_increase + step * 0.5; }
        if (being_honked_at) { stress_increase <- stress_increase + step * 1.5; }  // Honking is more stressful
        stress_level <- min(10.0, stress_level + stress_increase);
        road_rage_timer <- road_rage_timer + step;
        
        // Reset brake checking each step
        is_brake_checking <- false;
        is_nervous <- false;
        
        // TIMID DRIVERS - get nervous and erratic
        if (is_timid_driver) {
            is_nervous <- true;
            
            // Nervous behaviors:
            // 1. Become distracted from stress
            if (flip(0.15)) {
                is_distracted <- true;
                distraction_timer <- 0.0;
            }
            
            // 2. Erratic speed changes - speed up nervously then slow down
            if (flip(0.3)) {
                if (flip(0.5)) {
                    speed <- speed * (1.1 + rnd(0.2));  // Nervous acceleration
                } else {
                    speed <- speed * (0.7 + rnd(0.2));  // Nervous braking
                }
            }
            
            // 3. Try to get out of the way - move to side or slow way down
            if (stress_level > 3.0) {
                road my_road <- get_current_road();
                if (my_road != nil and my_road.num_lanes >= 2 and current_lane = 1 and lane_change_progress >= 1.0) {
                    // Move right to let them pass
                    if (is_lane_clear(0)) {
                        do do_lane_change(0);
                    }
                }
                // Or just slow down significantly to encourage passing
                if (flip(0.2)) {
                    speed <- speed * 0.5;
                }
            }
            
            // 4. Wobbly steering from nervousness (visual effect through lane drift)
            if (stress_level > 5.0 and flip(0.1)) {
                // Slight lane position variance would go here
                total_near_misses <- total_near_misses + 1;  // Count as near miss due to erratic driving
            }
        }
        // AGGRESSIVE DRIVERS - road rage response!
        else if (is_aggressive_driver) {
            // Aggressive responses:
            // 1. BRAKE CHECK - tap brakes to scare tailgater
            if (stress_level > 2.0 and flip(0.25)) {
                is_brake_checking <- true;
                speed <- speed * 0.4;  // Sharp brake
                total_near_misses <- total_near_misses + 1;
            }
            
            // 2. Slow down ON PURPOSE to annoy them
            if (stress_level > 4.0 and !is_brake_checking and flip(0.3)) {
                speed <- min(speed, max_speed * 0.4);  // Drive deliberately slow
            }
            
            // 3. Honk back!
            if (being_honked_at and flip(0.5)) {
                honking <- true;
            }
            
            // 4. Refuse to let them pass - block lane changes
            if (stress_level > 5.0) {
                road my_road <- get_current_road();
                if (my_road != nil and my_road.num_lanes >= 2) {
                    // Match their lane change attempts
                    loop v over: nearby_vehicles {
                        if (species(v) = car) {
                            car c <- car(v);
                            if (c.is_tailgating and v.lane_change_progress < 1.0) {
                                // Try to block by changing to the other lane
                                int other_lane <- (current_lane = 0) ? 1 : 0;
                                if (is_lane_clear(other_lane) and flip(0.4)) {
                                    do do_lane_change(other_lane);
                                    speed <- speed * 1.2;  // Speed up to block
                                }
                            }
                        }
                    }
                }
            }
            
            // 5. Extreme road rage - dangerous maneuvers
            if (road_rage_timer > 5.0 and stress_level > 7.0 and flip(0.1)) {
                // Sudden swerve or erratic behavior
                if (flip(0.5)) {
                    speed <- 0.0;  // Full stop (very dangerous)
                    is_stopped <- true;
                    total_forced_brakes <- total_forced_brakes + 1;
                } else {
                    speed <- max_speed * 1.3;  // Floor it
                }
                total_near_misses <- total_near_misses + 1;
            }
        }
        // NORMAL DRIVERS - mixed reactions
        else {
            // 1. Get slightly distracted by checking mirrors
            if (flip(0.08)) {
                is_distracted <- true;
                distraction_timer <- 0.0;
            }
            
            // 2. Speed up slightly to create space
            if (flip(0.2) and speed < max_speed) {
                speed <- min(speed * 1.15, max_speed * 1.1);
            }
            
            // 3. If really stressed, try to move over
            if (stress_level > 4.0) {
                road my_road <- get_current_road();
                if (my_road != nil and my_road.num_lanes >= 2 and lane_change_progress >= 1.0) {
                    int other_lane <- (current_lane = 0) ? 1 : 0;
                    if (is_lane_clear(other_lane)) {
                        do do_lane_change(other_lane);
                    }
                }
            }
            
            // 4. Occasional brake check if very annoyed
            if (stress_level > 6.0 and aggressiveness > 0.5 and flip(0.1)) {
                is_brake_checking <- true;
                speed <- speed * 0.5;
            }
        }
    }
    
    // Stress relief when not being tailgated
    reflex stress_recovery when: !being_tailgated and !being_honked_at and stress_level > 0 {
        stress_level <- max(0.0, stress_level - step * 0.3);
        if (stress_level < 1.0) {
            road_rage_timer <- 0.0;
            is_nervous <- false;
        }
    }
    
    // Multi-lane overtake (existing behavior, updated)
    reflex aggressive_overtake when: aggressiveness > 0.35 and car_ahead != nil and lane_change_progress >= 1.0 and current_lane = 0 and !in_opposing_lane {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes < 2) { return; }
        
        base_vehicle ahead <- base_vehicle(car_ahead);
        if (ahead != nil and distance_to_car_ahead < get_safety_distance() * 3.0 and ahead.speed < max_speed * 0.85) {
            patience <- patience + 2;
            
            bool oncoming_clear <- (oncoming_vehicle = nil or distance_to_oncoming > 25.0);
            
            if (is_lane_clear(1) and (my_road.is_one_way or oncoming_clear)) {
                do do_lane_change(1);
                wants_to_overtake <- true;
                is_cutting_off <- false;
                speed <- min(speed * 1.2, max_speed * 1.1);
            }
            else if (aggressiveness > 0.6 and patience > max_patience * 0.4) {
                bool semi_clear <- true;
                point my_pos <- get_visual_position();
                loop v over: nearby_vehicles {
                    road v_road <- v.get_current_road();
                    if (v_road = my_road and v.current_lane = 1) {
                        point their_pos <- v.get_visual_position();
                        if (my_pos distance_to their_pos < min_safety_distance) {
                            semi_clear <- false;
                        }
                    }
                }
                
                bool risky_overtake <- (aggressiveness > 0.85 and flip(0.4));
                if ((semi_clear or risky_overtake) and (my_road.is_one_way or oncoming_clear or risky_overtake)) {
                    do do_lane_change(1);
                    wants_to_overtake <- true;
                    is_cutting_off <- true;
                    patience <- 0;
                    speed <- min(speed * 1.4, max_speed * 1.3);
                    total_near_misses <- total_near_misses + 1;
                }
            }
        }
    }
    
    // Opposing lane overtake for single-lane roads - MORE LIKELY TO TRIGGER
    reflex opposing_lane_overtake when: aggressiveness > 0.5 and car_ahead != nil and !in_opposing_lane and lane_change_progress >= 1.0 {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes != 1 or my_road.is_one_way) { return; }
        
        base_vehicle ahead <- base_vehicle(car_ahead);
        if (ahead = nil) { return; }
        
        // LOWERED thresholds - overtake if stuck behind slower vehicle
        if (distance_to_car_ahead > get_safety_distance() * 3.0) { return; }
        if (ahead.speed > max_speed * 0.7) { return; }
        
        patience <- patience + 3;  // Faster patience buildup
        
        // Calculate required clear distance based on speeds
        float overtake_distance <- distance_to_car_ahead + 12.0;
        float time_to_overtake <- overtake_distance / max(0.5, max_speed - ahead.speed);
        float required_clear <- max_speed * time_to_overtake * 2.0;  // Reduced safety margin
        
        // Aggressive drivers need less clearance
        required_clear <- required_clear * (1.3 - aggressiveness * 0.4);
        
        bool can_overtake <- is_opposing_lane_clear(required_clear);
        
        // Very aggressive drivers will take more risks
        if (!can_overtake and aggressiveness > 0.75 and patience > max_patience * 0.5) {
            // Risky overtake - reduced clearance check
            can_overtake <- is_opposing_lane_clear(required_clear * 0.4);
            if (can_overtake) {
                total_near_misses <- total_near_misses + 1;
            }
        }
        
        // Lower patience threshold for overtaking
        if (can_overtake and patience > max_patience * 0.2 * (1.0 - aggressiveness)) {
            in_opposing_lane <- true;
            opposing_lane_timer <- 0.0;
            wants_to_overtake <- true;
            is_cutting_off <- (aggressiveness > 0.65);
            speed <- min(speed * 1.35, max_speed * 1.25);
            total_opposing_lane_overtakes <- total_opposing_lane_overtakes + 1;
            patience <- 0;
        }
    }
    
    // Complete opposing lane overtake - return to own lane
    reflex complete_opposing_overtake when: in_opposing_lane {
        road my_road <- get_current_road();
        if (my_road = nil) { 
            in_opposing_lane <- false; 
            opposing_lane_timer <- 0.0;
            return; 
        }
        
        // Check if we've passed the vehicle we were overtaking
        bool passed_target <- true;
        point my_pos <- get_visual_position();
        point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
        float road_len <- sqrt(road_dir.x * road_dir.x + road_dir.y * road_dir.y);
        road_dir <- {road_dir.x / road_len, road_dir.y / road_len};
        
        loop v over: nearby_vehicles {
            if (v.in_opposing_lane) { continue; }  // Skip other overtakers
            
            road v_road <- v.get_current_road();
            if (v_road = my_road) {
                point their_pos <- v.get_visual_position();
                point to_them <- their_pos - my_pos;
                float forward_dot <- to_them.x * road_dir.x + to_them.y * road_dir.y;
                float dist <- my_pos distance_to their_pos;
                
                // Vehicle is still ahead or too close behind
                if (forward_dot > -min_safety_distance * 1.5 and dist < min_safety_distance * 2.5) {
                    passed_target <- false;
                }
            }
        }
        
        // Merge back requirements
        float merge_gap_needed <- min_safety_distance * (aggressiveness > 0.7 ? 1.2 : 2.0);
        bool can_merge <- passed_target;
        
        // Check if there's space to merge back
        if (can_merge) {
            loop v over: nearby_vehicles {
                if (v.in_opposing_lane) { continue; }
                
                road v_road <- v.get_current_road();
                if (v_road = my_road) {
                    point their_pos <- v.get_visual_position();
                    float dist <- my_pos distance_to their_pos;
                    point to_them <- their_pos - my_pos;
                    float forward_dot <- to_them.x * road_dir.x + to_them.y * road_dir.y;
                    
                    // Check gap both ahead and behind
                    if (abs(forward_dot) < merge_gap_needed and dist < merge_gap_needed * 1.2) {
                        can_merge <- false;
                    }
                }
            }
        }
        
        // Aggressive merge if oncoming traffic approaching
        if (!can_merge and oncoming_vehicle != nil and distance_to_oncoming < 30.0) {
            // Force merge - dangerous!
            total_near_misses <- total_near_misses + 1;
            total_reactive_brakes <- total_reactive_brakes + 1;
            is_cutting_off <- true;
            can_merge <- true;
        }
        
        if (can_merge) {
            in_opposing_lane <- false;
            opposing_lane_timer <- 0.0;
            wants_to_overtake <- false;
            
            if (is_cutting_off) {
                speed <- min(speed * 1.2, max_speed);  // Quick merge
            }
        } else {
            // Keep overtaking speed up
            speed <- min(speed, max_speed * 1.15);
        }
    }
    
    reflex lane_return when: current_lane = 1 and lane_change_progress >= 1.0 and !in_opposing_lane {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes < 2) { current_lane <- 0; wants_to_overtake <- false; return; }
        
        if (car_ahead = nil or distance_to_car_ahead > get_safety_distance() * 2.0) {
            float required <- (aggressiveness > 0.7) ? min_safety_distance * 0.7 : min_safety_distance * 1.2;
            bool can_return <- true;
            
            point my_pos <- get_visual_position();
            loop v over: nearby_vehicles {
                road v_road <- v.get_current_road();
                if (v_road = my_road and v.current_lane = 0) {
                    point their_pos <- v.get_visual_position();
                    float dist <- my_pos distance_to their_pos;
                    if (dist < required * 2) {
                        can_return <- false;
                    }
                }
            }
            
            if (can_return or (aggressiveness > 0.75 and flip(0.3))) {
                do do_lane_change(0);
                wants_to_overtake <- false;
                if (!can_return) {
                    is_cutting_off <- true;
                    total_near_misses <- total_near_misses + 1;
                } else {
                    is_cutting_off <- false;
                }
            }
        }
    }
    
    reflex prefer_right_lane when: current_lane = 1 and !wants_to_overtake and car_ahead = nil and lane_change_progress >= 1.0 and !in_opposing_lane {
        road my_road <- get_current_road();
        if (my_road != nil and my_road.num_lanes = 2 and is_lane_clear(0)) {
            do do_lane_change(0);
        }
    }
    
    reflex follow_car when: car_ahead != nil and !wants_to_overtake and !emergency_brake and !in_opposing_lane {
        float req <- get_safety_distance() * (aggressiveness > 0.7 ? 0.5 : 0.9);
        if (distance_to_car_ahead <= req) {
            base_vehicle ahead <- base_vehicle(car_ahead);
            if (ahead != nil) { speed <- (ahead.is_stopped or ahead.speed < 0.1) ? 0.0 : max(0.0, ahead.speed * 0.9); if (speed = 0.0) { is_stopped <- true; } }
        } else if (distance_to_car_ahead <= req * 1.5) {
            base_vehicle ahead <- base_vehicle(car_ahead);
            if (ahead != nil) { speed <- min(speed, min(ahead.speed, max_speed * 0.8)); }
        }
    }
    
    reflex react_to_light when: nearest_light != nil and !emergency_brake and (car_ahead = nil or distance_to_car_ahead > get_safety_distance()) {
        float dist <- location distance_to nearest_light.stop_position;
        string c <- nearest_light.get_current_color();
        float safe_dist <- dist - 1.0;
        
        if (c = "red") {
            // AGGRESSIVE: Run red lights sometimes!
            if (is_aggressive_driver and safe_dist < 8.0 and safe_dist > 0 and speed > max_speed * 0.6) {
                if (flip(0.4)) {  // 40% chance to run red
                    speed <- min(speed * 1.15, max_speed * 1.4);
                    total_near_misses <- total_near_misses + 1;
                    is_cutting_off <- true;
                } else {
                    speed <- max(0.0, speed * 0.5);  // Hard brake
                    total_reactive_brakes <- total_reactive_brakes + 1;
                }
            }
            // TIMID: Stop very early
            else if (is_timid_driver) {
                if (safe_dist <= 2.0) { speed <- 0.0; is_stopped <- true; }
                else if (safe_dist < 30.0) { speed <- min(speed, max(0.0, (safe_dist / 30.0) * max_speed * 0.3)); }
            }
            // Normal: standard behavior with some variance
            else {
                if (safe_dist <= 0.5) { speed <- 0.0; is_stopped <- true; }
                else if (safe_dist < 20.0) { speed <- min(speed, max(0.0, (safe_dist / 20.0) * max_speed * 0.5)); }
                else { speed <- min(speed, (safe_dist / perception_distance) * max_speed * 0.6); }
            }
        } else if (c = "yellow") {
            // AGGRESSIVE: Always gun it on yellow
            if (is_aggressive_driver and safe_dist < 25.0 and safe_dist > 1.0) {
                speed <- min(speed * 1.4, max_speed * 1.5);
            }
            // TIMID: Always stop on yellow
            else if (is_timid_driver) {
                if (safe_dist > 5.0) { speed <- min(speed, (safe_dist / 25.0) * max_speed * 0.3); }
                else { speed <- 0.0; is_stopped <- true; }
            }
            // Normal: 50/50 decision
            else {
                if (flip(0.5) and safe_dist < 15.0 and safe_dist > 2.0) { speed <- min(speed * 1.2, max_speed * 1.3); }
                else if (safe_dist > 10.0) { speed <- min(speed, (safe_dist / 20.0) * max_speed * 0.4); }
                else if (safe_dist <= 0.5) { speed <- 0.0; is_stopped <- true; }
            }
        } else {
            is_stopped <- false;
            // Variable acceleration from stop
            float accel <- 0.2 + aggressiveness * 0.4 + rnd(0.2);
            speed <- min(speed + accel, max_speed);
        }
    }
    
    reflex accelerate when: !is_stopped and !emergency_brake and nearest_light = nil and car_ahead = nil and oncoming_vehicle = nil and threat_vehicle = nil and !in_opposing_lane {
        // CHAOTIC: Variable acceleration with random fluctuations
        float accel <- 0.15 + aggressiveness * 0.35;
        
        // Random speed variance (humans don't maintain constant speed)
        float random_factor <- 1.0 + (rnd(speed_variance * 2) - speed_variance);
        
        // Aggressive drivers accelerate harder
        if (is_aggressive_driver) {
            accel <- accel * 1.5;
        }
        // Timid drivers accelerate gently
        if (is_timid_driver) {
            accel <- accel * 0.5;
        }
        
        speed <- min(speed + accel, max_speed * random_factor);
        
        // Random slight braking (humans are inconsistent)
        if (flip(0.03) and !is_aggressive_driver) {
            speed <- speed * (0.85 + rnd(0.1));
        }
    }
    
    reflex move when: !is_stopped and !emergency_brake { 
        do drive; 
        if (final_target = nil) { 
            cars_completed <- cars_completed + 1;
            total_car_travel_time <- total_car_travel_time + (time - spawn_time);
            do die; 
        } 
    }
    
    reflex check_resume when: is_stopped and !emergency_brake {
        bool can <- (nearest_light = nil or nearest_light.get_current_color() = "green");
        if (car_ahead != nil and distance_to_car_ahead <= get_safety_distance()) {
            base_vehicle ahead <- base_vehicle(car_ahead);
            if (ahead != nil and ahead.is_stopped) { can <- false; }
        }
        road my_road <- get_current_road();
        if (my_road != nil and !my_road.is_one_way and my_road.num_lanes = 1 and oncoming_vehicle != nil and distance_to_oncoming < 12.0) {
            can <- false;
        }
        if (threat_vehicle != nil and threat_distance < min_safety_distance) {
            can <- false;
        }
        if (can) { 
            is_stopped <- false; 
            speed <- 0.5; 
            do drive; 
            if (final_target = nil) { 
                cars_completed <- cars_completed + 1;
                total_car_travel_time <- total_car_travel_time + (time - spawn_time);
                do die; 
            } 
        }
    }
    
    reflex reset_patience when: car_ahead = nil or distance_to_car_ahead > get_safety_distance() * 3 {
        patience <- max(0, patience - 1); 
        if (car_ahead = nil) { honking <- false; }
    }
    
    aspect default {
        point draw_loc <- get_visual_position();
        
        rgb dc <- #blue;
        if (emergency_brake) { dc <- #red; }
        else if (is_brake_checking) { dc <- #darkred; }  // Brake checking - dark red
        else if (in_opposing_lane) { dc <- #crimson; }
        else if (reactive_brake) { dc <- #maroon; }
        else if (is_nervous) { dc <- #lightyellow; }  // Nervous driver - light yellow
        else if (is_stopped) { dc <- #darkblue; }
        else if (is_cutting_off) { dc <- #orange; }
        else if (is_tailgating) { dc <- #cyan; }
        else if (being_tailgated and stress_level > 3.0) { dc <- #mediumpurple; }  // Stressed
        else if (lane_change_progress < 1.0) { dc <- #yellow; }
        else if (oncoming_vehicle != nil and distance_to_oncoming < 18.0) { dc <- #magenta; }
        else if (threat_vehicle != nil) { dc <- #pink; }
        
        draw circle(1.5) at: draw_loc color: dc border: #black;
        if (aggressiveness > 0.6) { draw circle(0.6) at: draw_loc color: #red; }
        if (honking) { draw circle(3.0) at: draw_loc color: rgb(255, 255, 0, 100) border: #yellow; }
        
        // Show stress level indicator when being tailgated
        if (stress_level > 2.0) {
            float stress_alpha <- min(180.0, stress_level * 20);
            draw circle(2.5 + stress_level * 0.2) at: draw_loc color: rgb(255, 100, 100, stress_alpha) border: rgb(255, 0, 0, stress_alpha);
        }
        
        // Show opposing lane indicator
        if (in_opposing_lane) {
            draw triangle(2.0) at: draw_loc color: #white rotate: heading;
            draw circle(4.0) at: draw_loc color: rgb(255, 0, 0, 60) border: #red;
        } else if (current_lane = 1) {
            draw triangle(1.0) at: draw_loc color: #white rotate: heading;
        }
    }
}

species autonomous_car parent: base_vehicle {
    rgb color <- #purple;
    float max_speed <- 5.0;
    list<autonomous_car> nearby_autonomous <- [];
    map<traffic_light, string> shared_light_status <- [];
    map<road, int> shared_road_congestion <- [];
    list<road> congested_roads <- [];
    int reroute_count <- 0;
    float surrounding_aggressiveness <- 0.0;
    int aggressive_drivers_nearby <- 0;
    bool defensive_mode <- false;
    float speed_adjustment <- 1.0;
    map<base_vehicle, float> driver_speed_history <- [];
    map<base_vehicle, bool> predicted_lane_change <- [];
    int cycles_since_reroute <- 0;
    
    // COOPERATIVE DRIVING ATTRIBUTES
    bool in_platoon <- false;
    autonomous_car platoon_leader <- nil;
    list<autonomous_car> platoon_members <- [];
    float target_speed <- 5.0;  // Smooth speed target
    float platoon_spacing <- 3.0;  // Consistent spacing in platoon
    bool coordinating_lane_change <- false;
    int platoon_position <- 0;  // Position in platoon (0 = leader)
    float smooth_accel_rate <- 0.3;  // Gradual acceleration
    float smooth_decel_rate <- 0.25;  // Gradual deceleration
    float spawn_time <- 0.0;  // Track when spawned for throughput
    
    // SMART ROUTE PLANNING ATTRIBUTES
    map<intersection, float> signal_timing_cache <- [];  // Cache of signal wait times
    float last_route_evaluation <- 0.0;
    float route_eval_interval <- 15.0;  // Re-evaluate route periodically
    bool using_signal_aware_route <- false;
    float estimated_arrival_time <- 0.0;
    list<intersection> upcoming_signals <- [];  // Signals on current route
    map<road, float> road_speed_estimates <- [];  // Estimated speeds on roads
    
    action initialize_trip {
        start_node <- one_of(spawn_node where each.is_start);
        target_node <- one_of(spawn_node where each.is_end);
        location <- start_node.location;
        spawn_time <- time;  // Record spawn time
        min_safety_distance <- 4.0;  // Consistent safe distance
        safety_distance_factor <- 0.6;  // Predictable
        lane_change_speed <- 0.06;  // SMOOTH, slower lane changes
        current_lane <- 0;
        target_speed <- max_speed;
        last_route_evaluation <- time;
        
        // Use smart signal-aware routing from the start
        do smart_reroute;
    }
    
    // COOPERATIVE: Form platoons with nearby AVs
    reflex form_platoon {
        if (!in_platoon) {
            // Look for AV ahead to join platoon
            loop av over: nearby_autonomous {
                //if (av.in_platoon or av = platoon_leader) { continue; }
                
                road my_road <- get_current_road();
                road av_road <- av.get_current_road();
                
                if (my_road = av_road and my_road != nil) {
                    point my_pos <- get_visual_position();
                    point av_pos <- av.get_visual_position();
                    float dist <- my_pos distance_to av_pos;
                    
                    point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
                    point to_av <- av_pos - my_pos;
                    float forward <- to_av.x * road_dir.x + to_av.y * road_dir.y;
                    
                    // AV is ahead and close enough to platoon
                    if (forward > 0 and dist < 12.0 and dist > platoon_spacing) {
                        in_platoon <- true;
                        platoon_leader <- av;
                        platoon_position <- av.platoon_position + 1;
                        
                        // Share target speed with platoon
                        target_speed <- av.target_speed;
                        
                        // Notify leader
                        if (av.platoon_members = nil) {
                            av.platoon_members <- [];
                        }
                        av.platoon_members <- av.platoon_members + self;
                    }
                }
            }
        } else {
            // Maintain platoon or leave if leader gone
            if (platoon_leader = nil or dead(platoon_leader)) {
                in_platoon <- false;
                platoon_leader <- nil;
                platoon_position <- 0;
            }
        }
    }
    
    reflex communicate {
        nearby_autonomous <- autonomous_car where (location distance_to each.location <= 80.0 and each != self);
        if (nearest_light != nil) {
            shared_light_status[nearest_light] <- nearest_light.get_current_color();
            loop other over: nearby_autonomous { other.shared_light_status[nearest_light] <- shared_light_status[nearest_light]; }
        }
        road my_road <- get_current_road();
        if (my_road != nil) {
            loop other over: nearby_autonomous { other.shared_road_congestion[my_road] <- my_road.current_vehicle_count; }
        }
        loop r over: shared_road_congestion.keys { if (shared_road_congestion[r] > 2) { congested_roads << r; } }
    }
    
    reflex perceive {
        do scan_nearby_vehicles;
        do find_traffic_light;
        do find_car_ahead;
        do find_oncoming_traffic;
        do detect_incoming_threats;
    }
    
    reflex analyze_drivers {
        aggressive_drivers_nearby <- 0;
        surrounding_aggressiveness <- 0.0;
        int human_count <- 0;
        
        loop v over: nearby_vehicles {
            if (v is car) {
                car h <- car(v);
                human_count <- human_count + 1;
                surrounding_aggressiveness <- surrounding_aggressiveness + h.aggressiveness;
                if (h.aggressiveness > 0.6) { aggressive_drivers_nearby <- aggressive_drivers_nearby + 1; }
                driver_speed_history[v] <- v.speed;
                predicted_lane_change[v] <- (h.is_tailgating or h.patience > h.max_patience / 2 or h.in_opposing_lane);
            }
        }
        if (human_count > 0) { surrounding_aggressiveness <- surrounding_aggressiveness / human_count; }
        defensive_mode <- aggressive_drivers_nearby >= 1 or surrounding_aggressiveness > 0.5;
        speed_adjustment <- defensive_mode ? 0.8 : 1.0;
        min_safety_distance <- defensive_mode ? 6.0 : 5.0;
        safety_distance_factor <- defensive_mode ? 0.9 : 0.7;
    }
    
    reflex check_congestion {
        cycles_since_reroute <- cycles_since_reroute + 1;
        
        if (current_path = nil or empty(current_path.edges)) { return; }
        
        float route_congestion <- 0.0;
        int checked <- 0;
        int max_count <- 0;
        
        loop edge over: current_path.edges {
            road r <- road(edge);
            if (r != nil) {
                route_congestion <- route_congestion + r.current_vehicle_count;
                checked <- checked + 1;
                if (r.current_vehicle_count > max_count) {
                    max_count <- r.current_vehicle_count;
                }
                if (r.is_congested) {
                    max_count <- max(max_count, r.capacity);
                }
            }
        }
        if (checked > 0) { route_congestion <- route_congestion / checked; }
        
        bool should_reroute <- (max_count >= 3) or (route_congestion > 2.0) or (cycles_since_reroute > 100 and route_congestion > 1.0);
        
        // Also reroute if signal timing suggests better route available
        if (!should_reroute and (time - last_route_evaluation) > route_eval_interval) {
            should_reroute <- true;
        }
        
        if (should_reroute and target_node != nil and cycles_since_reroute > 20) {
            do smart_reroute;
        }
    }
    
    // SMART SIGNAL-AWARE ROUTING
    action smart_reroute {
        agent source_node <- nil;
        float min_dist <- 1000.0;
        
        loop inter over: intersection {
            float d <- location distance_to inter.location;
            if (d < min_dist) { min_dist <- d; source_node <- inter; }
        }
        loop sn over: spawn_node {
            float d <- location distance_to sn.location;
            if (d < min_dist) { min_dist <- d; source_node <- sn; }
        }
        
        if (source_node = nil) { return; }
        
        // Calculate weights considering traffic AND signal timing
        map<road, float> weights <- [];
        loop r over: road {
            float base_weight <- r.shape.perimeter;  // Distance-based
            
            // TRAFFIC PENALTY
            float capacity_ratio <- r.current_vehicle_count / max(1, r.capacity);
            float congestion_penalty <- capacity_ratio * 15.0;
            
            if (!r.is_one_way and r.num_lanes = 1) {
                congestion_penalty <- congestion_penalty + 8.0;
            }
            
            if (shared_road_congestion contains_key r) {
                congestion_penalty <- congestion_penalty + shared_road_congestion[r] * 4.0;
            }
            if (congested_roads contains r) {
                congestion_penalty <- congestion_penalty + 20.0;
            }
            if (r.is_congested) {
                congestion_penalty <- congestion_penalty + 25.0;
            }
            
            // SIGNAL TIMING PENALTY - estimate wait time at this road's intersection
            float signal_penalty <- 0.0;
            if (r.target_node != nil and r.target_node is intersection) {
                intersection target_inter <- intersection(r.target_node);
                if (target_inter.has_traffic_signal) {
                    float wait_time <- target_inter.time_to_green(r.direction);
                    signal_penalty <- wait_time * 2.0;  // Weight signal waits heavily
                    
                    // Bonus for green waves - if currently green with time remaining
                    float green_remaining <- target_inter.remaining_green_time(r.direction);
                    if (green_remaining > 3.0) {
                        signal_penalty <- signal_penalty - 5.0;  // Favor green routes
                    }
                }
            }
            
            // ESTIMATED SPEED on this road segment
            float avg_speed <- max_speed;
            if (r.current_vehicle_count > 0) {
                list<base_vehicle> on_road <- base_vehicle where (each.get_current_road() = r);
                if (!empty(on_road)) {
                    avg_speed <- mean(on_road collect each.speed);
                    if (avg_speed < max_speed * 0.5) {
                        congestion_penalty <- congestion_penalty + 10.0;  // Slow traffic penalty
                    }
                }
            }
            
            weights[r] <- base_weight + congestion_penalty + max(0.0, signal_penalty);
        }
        
        graph weighted <- as_driving_graph(road, list<agent>(intersection) + list<agent>(spawn_node)) with_weights weights;
        do compute_path(graph: weighted, source: source_node, target: target_node);
        reroute_count <- reroute_count + 1;
        cycles_since_reroute <- 0;
        last_route_evaluation <- time;
        using_signal_aware_route <- true;
        
        // Update upcoming signals list for speed adjustment
        do update_upcoming_signals;
    }
    
    // Track signals on current route for proactive speed adjustment
    action update_upcoming_signals {
        upcoming_signals <- [];
        if (current_path = nil or empty(current_path.edges)) { return; }
        
        loop edge over: current_path.edges {
            road r <- road(edge);
            if (r != nil and r.target_node != nil and r.target_node is intersection) {
                intersection inter <- intersection(r.target_node);
                if (inter.has_traffic_signal and !(upcoming_signals contains inter)) {
                    upcoming_signals <- upcoming_signals + inter;
                }
            }
        }
    }
    
    // SIGNAL-AWARE SPEED CONTROL - adjust speed to hit green lights
    reflex signal_aware_speed when: !empty(upcoming_signals) {
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        // Find the next signal on our route
        intersection next_signal <- nil;
        float dist_to_signal <- 1000.0;
        
        loop inter over: upcoming_signals {
            float d <- location distance_to inter.location;
            if (d < dist_to_signal and d < perception_distance * 2) {
                next_signal <- inter;
                dist_to_signal <- d;
            }
        }
        
        if (next_signal = nil) { return; }
        
        // Determine what direction we'll approach from
        string approach_dir <- my_road.direction;
        
        float time_to_green <- next_signal.time_to_green(approach_dir);
        float green_remaining <- next_signal.remaining_green_time(approach_dir);
        
        // Calculate time to reach signal at current speed
        float time_to_arrive <- (speed > 0.1) ? dist_to_signal / speed : 100.0;
        
        if (green_remaining > 0) {
            // Signal is green - check if we can make it
            if (time_to_arrive < green_remaining - 1.0) {
                // Can make this green, maintain or slightly increase speed
                target_speed <- min(max_speed, speed * 1.1);
            } else if (time_to_arrive < green_remaining + 2.0) {
                // Borderline - speed up a bit to catch the green
                target_speed <- min(max_speed * 1.15, speed * 1.2);
            } else {
                // Won't make this green - slow down to catch next green
                float full_cycle <- next_signal.time_to_change * 2 + next_signal.yellow_time * 2;
                float optimal_arrival <- green_remaining + full_cycle - 2.0;
                if (optimal_arrival > 0 and dist_to_signal > 0) {
                    target_speed <- max(max_speed * 0.4, dist_to_signal / optimal_arrival);
                }
            }
        } else {
            // Signal is red/yellow - plan for when it turns green
            if (time_to_green < 3.0 and dist_to_signal < 15.0) {
                // Almost green, slow approach
                target_speed <- max_speed * 0.6;
            } else if (time_to_green > 0) {
                // Calculate speed to arrive just after green
                float optimal_arrival <- time_to_green + 1.0;
                if (optimal_arrival > 0 and dist_to_signal > 5) {
                    float ideal_speed <- dist_to_signal / optimal_arrival;
                    // Don't go too slow, but try to time it
                    target_speed <- max(max_speed * 0.5, min(max_speed, ideal_speed));
                }
            }
        }
    }
    
    reflex handle_oncoming when: oncoming_vehicle != nil {
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        
        if (!my_road.is_one_way and my_road.num_lanes = 1) {
            if (distance_to_oncoming < 25.0) {
                total_reactive_brakes <- total_reactive_brakes + 1;
                speed <- min(speed, max_speed * 0.3);
                if (distance_to_oncoming < 12.0) {
                    total_near_misses <- total_near_misses + 1;
                    speed <- 0.0;
                    is_stopped <- true;
                }
            }
        }
    }
    
    // React to vehicles in opposing lane - yield to let them merge back safely
    reflex yield_to_opposing_lane_vehicles {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes != 1 or my_road.is_one_way) { return; }
        
        point my_pos <- get_visual_position();
        point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
        float road_len <- sqrt(road_dir.x * road_dir.x + road_dir.y * road_dir.y);
        road_dir <- {road_dir.x / road_len, road_dir.y / road_len};
        
        loop v over: nearby_vehicles {
            if (v.in_opposing_lane) {
                point their_pos <- v.get_visual_position();
                float dist <- my_pos distance_to their_pos;
                
                if (dist < min_safety_distance * 6) {
                    point to_them <- their_pos - my_pos;
                    float forward_dot <- to_them.x * road_dir.x + to_them.y * road_dir.y;
                    
                    // They're ahead or beside - create gap for them to merge
                    if (forward_dot > -min_safety_distance * 2) {
                        speed <- speed * 0.5;
                        total_reactive_brakes <- total_reactive_brakes + 1;
                        
                        // If very close, stop to let them in
                        if (dist < min_safety_distance * 2.5 and forward_dot > 0) {
                            speed <- 0.0;
                            is_stopped <- true;
                            total_near_misses <- total_near_misses + 1;
                        }
                    }
                }
            }
        }
    }
    
    reflex react_to_threat when: threat_vehicle != nil {
        if (reactive_brake) {
            total_near_misses <- total_near_misses + 1;
            total_reactive_brakes <- total_reactive_brakes + 1;
            
            if (threat_distance < collision_radius * 4) {
                total_forced_brakes <- total_forced_brakes + 1;
                speed <- 0.0;
                is_stopped <- true;
                emergency_brake <- true;
            } else if (threat_distance < min_safety_distance * 1.5) {
                total_forced_brakes <- total_forced_brakes + 1;
                speed <- speed * 0.4;
            } else {
                speed <- speed * 0.7;
            }
        } else if (threat_distance < min_safety_distance * 2.5) {
            speed <- speed * 0.85;
        }
    }
    
    reflex collision_avoidance {
        emergency_brake <- false;
        point my_pos <- get_visual_position();
        
        loop v over: nearby_vehicles {
            point their_pos <- v.get_visual_position();
            float dist <- my_pos distance_to their_pos;
            float combined <- collision_radius + v.collision_radius;
            if (dist < combined + 3.0) { 
                emergency_brake <- true; 
                speed <- 0.0; 
                is_stopped <- true; 
                total_forced_brakes <- total_forced_brakes + 1; 
                total_reactive_brakes <- total_reactive_brakes + 1;
                total_near_misses <- total_near_misses + 1;
                return; 
            }
            
            if (v is car) {
                car h <- car(v);
                if (h.is_tailgating) {
                    point to_us <- my_pos - their_pos;
                    if (to_us.x * cos(v.heading) + to_us.y * sin(v.heading) > 0 and dist < 12.0) {
                        road my_road <- get_current_road();
                        if (my_road != nil and my_road.num_lanes = 2 and lane_change_progress >= 1.0) {
                            if (current_lane = 1 and is_lane_clear(0)) { 
                                do do_lane_change(0); 
                            }
                        }
                    }
                }
                if (predicted_lane_change contains_key v and predicted_lane_change[v] = true) {
                    road my_road <- get_current_road();
                    road v_road <- v.get_current_road();
                    if (my_road = v_road and my_road != nil and my_road.num_lanes = 2 and dist < min_safety_distance * 4) {
                        speed <- max(0.5, speed * 0.85);
                        total_reactive_brakes <- total_reactive_brakes + 1;
                    }
                }
                
                if (h.is_cutting_off and dist < min_safety_distance * 2.5) {
                    total_near_misses <- total_near_misses + 1;
                    total_reactive_brakes <- total_reactive_brakes + 1;
                    speed <- speed * 0.5;
                    if (dist < min_safety_distance * 1.5) {
                        total_forced_brakes <- total_forced_brakes + 1;
                        speed <- speed * 0.3;
                    }
                }
                
                // React to car in opposing lane
                if (h.in_opposing_lane and dist < min_safety_distance * 4) {
                    total_reactive_brakes <- total_reactive_brakes + 1;
                    speed <- speed * 0.4;
                }
            }
        }
        if (check_collision_risk()) { 
            speed <- max(0.0, speed * 0.5);
            total_reactive_brakes <- total_reactive_brakes + 1; 
        }
    }
    
    reflex yield_aggressive when: defensive_mode {
        road my_road <- get_current_road();
        if (my_road = nil) { return; }
        point my_pos <- get_visual_position();
        
        loop v over: nearby_vehicles where (each is car) {
            car h <- car(v);
            if (h.aggressiveness > 0.7 and (h.patience > h.max_patience * 0.6 or h.in_opposing_lane)) {
                point their_pos <- v.get_visual_position();
                float dist <- my_pos distance_to their_pos;
                point to_us <- my_pos - their_pos;
                point road_dir <- last(my_road.shape.points) - first(my_road.shape.points);
                if (to_us.x * road_dir.x + to_us.y * road_dir.y > 0 and dist < 18.0) {
                    speed <- max(speed * 0.8, max_speed * 0.4);
                    total_reactive_brakes <- total_reactive_brakes + 1;
                    if (my_road.num_lanes = 2 and current_lane = 1 and is_lane_clear(0) and lane_change_progress >= 1.0) {
                        do do_lane_change(0);
                    }
                }
            }
        }
    }
    
    reflex safe_lane_change when: car_ahead != nil and !defensive_mode and lane_change_progress >= 1.0 and current_lane = 0 {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes < 2) { return; }
        base_vehicle ahead <- base_vehicle(car_ahead);
        if (ahead != nil and distance_to_car_ahead < get_safety_distance() * 2.5 and ahead.speed < speed * 0.7) {
            bool safe <- is_lane_clear(1);
            if (!my_road.is_one_way and oncoming_vehicle != nil and distance_to_oncoming < 40.0) {
                safe <- false;
            }
            loop v over: nearby_vehicles {
                if (predicted_lane_change contains_key v and predicted_lane_change[v] = true) {
                    road v_road <- v.get_current_road();
                    if (v_road = my_road) { safe <- false; }
                }
                // Don't change lanes if someone is in opposing lane nearby
                if (v.in_opposing_lane) {
                    road v_road <- v.get_current_road();
                    if (v_road = my_road) {
                        point their_pos <- v.get_visual_position();
                        if (get_visual_position() distance_to their_pos < min_safety_distance * 5) {
                            safe <- false;
                        }
                    }
                }
            }
            if (safe) { do do_lane_change(1); wants_to_overtake <- true; }
        }
    }
    
    reflex return_lane when: current_lane = 1 and lane_change_progress >= 1.0 {
        road my_road <- get_current_road();
        if (my_road = nil or my_road.num_lanes < 2) { current_lane <- 0; wants_to_overtake <- false; return; }
        if ((car_ahead = nil or distance_to_car_ahead > get_safety_distance() * 4.0) and is_lane_clear(0)) {
            do do_lane_change(0);
            wants_to_overtake <- false;
        }
    }
    
    reflex prefer_right_lane when: current_lane = 1 and !wants_to_overtake and car_ahead = nil and lane_change_progress >= 1.0 {
        road my_road <- get_current_road();
        if (my_road != nil and my_road.num_lanes = 2 and is_lane_clear(0)) {
            do do_lane_change(0);
        }
    }
    
    reflex follow_car when: car_ahead != nil and !wants_to_overtake and !emergency_brake {
        float req <- get_safety_distance();
        base_vehicle ahead <- base_vehicle(car_ahead);
        
        if (ahead != nil) {
            // COOPERATIVE: Smooth speed adjustment (no jerky braking)
            float target <- ahead.speed;
            
            // In platoon: maintain consistent spacing
            if (in_platoon and ahead is autonomous_car) {
                float ideal_gap <- platoon_spacing + platoon_position * 0.5;
                if (distance_to_car_ahead < ideal_gap) {
                    // Gently slow to open gap
                    target <- max(0.0, ahead.speed - smooth_decel_rate);
                } else if (distance_to_car_ahead > ideal_gap * 1.5) {
                    // Gently speed up to close gap
                    target <- min(ahead.speed + smooth_accel_rate, max_speed);
                } else {
                    // Perfect spacing - match speed exactly
                    target <- ahead.speed;
                }
            }
            // Not in platoon: standard smooth following
            else {
                if (distance_to_car_ahead <= req * 0.8) {
                    target <- (ahead.is_stopped or ahead.speed < 0.1) ? 0.0 : ahead.speed * 0.9;
                } else if (distance_to_car_ahead <= req) {
                    target <- ahead.speed * 0.95;
                } else if (distance_to_car_ahead <= req * 1.5) {
                    target <- min(ahead.speed * 1.05, max_speed * 0.85);
                }
            }
            
            // SMOOTH speed transition (never jerk)
            if (speed > target) {
                speed <- max(target, speed - smooth_decel_rate);
            } else {
                speed <- min(target, speed + smooth_accel_rate);
            }
            
            if (speed < 0.1 and ahead.is_stopped) {
                speed <- 0.0;
                is_stopped <- true;
            }
        }
    }
    
    reflex react_to_light when: nearest_light != nil and !emergency_brake and (car_ahead = nil or distance_to_car_ahead > get_safety_distance()) {
        float dist <- location distance_to nearest_light.stop_position;
        string c <- nearest_light.get_current_color();
        float safe_dist <- dist - 1.5;
        float eff_max <- max_speed * speed_adjustment;
        
        if (c = "red") {
            if (safe_dist <= 0.5) { speed <- 0.0; is_stopped <- true; }
            else if (safe_dist < 25.0) { speed <- min(speed, max(0.0, (safe_dist / 25.0) * eff_max * 0.5)); }
            else { speed <- min(speed, eff_max * 0.7); }
        } else if (c = "yellow") {
            if (safe_dist > 15.0) { speed <- min(speed, (safe_dist / 25.0) * eff_max * 0.4); }
            else if (safe_dist <= 0.5) { speed <- 0.0; is_stopped <- true; }
            else { speed <- min(speed * 1.02, eff_max); }
        } else {
            is_stopped <- false;
            if (dist < 25.0 and nearest_light.my_intersection.counter > nearest_light.my_intersection.time_to_change - 1.0) { speed <- min(speed, eff_max * 0.9); }
            else { speed <- min(speed + 0.4, eff_max); }
        }
    }
    
    reflex accelerate when: !is_stopped and !emergency_brake and nearest_light = nil and car_ahead = nil and oncoming_vehicle = nil and threat_vehicle = nil {
        // COOPERATIVE: Smooth, consistent acceleration
        float target <- max_speed * speed_adjustment;
        
        // In platoon: match leader speed
        if (in_platoon and platoon_leader != nil) {
            target <- platoon_leader.target_speed * speed_adjustment;
        }
        
        // Smooth acceleration (never sudden)
        if (speed < target) {
            speed <- min(target, speed + smooth_accel_rate);
        } else if (speed > target) {
            speed <- max(target, speed - smooth_decel_rate);
        }
        
        // Update target for followers
        target_speed <- speed;
    }
    
    reflex move when: !is_stopped and !emergency_brake { 
        do drive; 
        if (final_target = nil) { 
            avs_completed <- avs_completed + 1;
            total_av_travel_time <- total_av_travel_time + (time - spawn_time);
            do die; 
        } 
    }

    reflex check_resume when: is_stopped and !emergency_brake {
        bool can <- (nearest_light = nil or nearest_light.get_current_color() = "green");
        if (car_ahead != nil and distance_to_car_ahead <= get_safety_distance()) {
            base_vehicle ahead <- base_vehicle(car_ahead);
            if (ahead != nil and ahead.is_stopped) { can <- false; }
        }
        road my_road <- get_current_road();
        if (my_road != nil and !my_road.is_one_way and my_road.num_lanes = 1 and oncoming_vehicle != nil and distance_to_oncoming < 15.0) {
            can <- false;
        }
        if (threat_vehicle != nil and threat_distance < min_safety_distance) {
            can <- false;
        }
        // Don't resume if vehicle in opposing lane is nearby
        loop v over: nearby_vehicles {
            if (v.in_opposing_lane) {
                point their_pos <- v.get_visual_position();
                if (get_visual_position() distance_to their_pos < min_safety_distance * 2.5) {
                    can <- false;
                }
            }
        }
        if (can) { 
            is_stopped <- false; 
            speed <- 0.5; 
            do drive; 
            if (final_target = nil) { 
                avs_completed <- avs_completed + 1;
                total_av_travel_time <- total_av_travel_time + (time - spawn_time);
                do die; 
            } 
        }
    }
    
    aspect default {
        point draw_loc <- get_visual_position();
        
        rgb dc <- emergency_brake ? #red : (is_stopped ? #indigo : (defensive_mode ? #teal : color));
        if (reactive_brake) { dc <- #maroon; }
        if (lane_change_progress < 1.0) { dc <- #lime; }
        if (in_platoon) { dc <- #darkviolet; }  // Platoon color
        
        draw circle(1.5) at: draw_loc color: dc border: #black;
        
        // Show platoon connection
        if (in_platoon and platoon_leader != nil and !dead(platoon_leader)) {
            draw line([draw_loc, platoon_leader.get_visual_position()]) color: rgb(150, 0, 255, 100) width: 1.0;
        }
        
        if (defensive_mode) { draw circle(8.0) at: draw_loc color: rgb(0, 255, 255, 50) border: rgb(0, 255, 255, 100); }
        
        if (cycles_since_reroute < 10) {
            draw circle(4.0) at: draw_loc color: rgb(0, 255, 0, 80) border: #green;
        }
        
        if (current_lane = 1) {
            draw triangle(1.0) at: draw_loc color: #white rotate: heading;
        }
        
        if (threat_vehicle != nil and threat_distance < min_safety_distance * 2.5) {
            draw circle(5.0) at: draw_loc color: rgb(255, 100, 100, 60) border: #red;
        }
    }
}

// ==================== EXPERIMENTS ====================

experiment RegularCarsOnly type: gui {
    init { spawn_regular_cars <- true; spawn_autonomous_cars <- false; }
    parameter "Number of Cars" var: nb_cars min: 1 max: 100;
    
    output {
        display "Human Drivers" background: #white type: 2d axes: false autosave: false {
            species road; species intersection; species traffic_light; species spawn_node; species car;
        }
        display "Speed" type: 2d refresh: every(2 #cycles) {
            chart "Speed Distribution" type: series {
                data "Average" value: (empty(car) ? 0.0 : mean(car collect each.speed)) color: #blue;
                data "Max" value: (empty(car) ? 0.0 : max(car collect each.speed)) color: #green;
                data "Min" value: (empty(car) ? 0.0 : min(car collect each.speed)) color: #red;
            }
        }
        display "Throughput" type: 2d refresh: every(2 #cycles) {
            chart "Traffic Throughput" type: series {
                data "Trips Completed" value: cars_completed color: #blue;
                data "Avg Travel Time" value: (cars_completed > 0 ? total_car_travel_time / cars_completed : 0.0) color: #orange;
                data "Active Vehicles" value: length(car) color: #green;
            }
        }
    }
}

experiment AutonomousCarsOnly type: gui {
    init { spawn_regular_cars <- false; spawn_autonomous_cars <- true; }
    parameter "Number of AVs" var: nb_autonomous_cars min: 1 max: 100;
    
    output {
        display "Autonomous Vehicles" background: #white type: 2d axes: false autosave: false {
            species road; species intersection; species traffic_light; species spawn_node; species autonomous_car;
        }
        display "Speed" type: 2d refresh: every(2 #cycles) {
            chart "Speed Distribution" type: series {
                data "Average" value: (empty(autonomous_car) ? 0.0 : mean(autonomous_car collect each.speed)) color: #purple;
                data "Max" value: (empty(autonomous_car) ? 0.0 : max(autonomous_car collect each.speed)) color: #blue;
                data "Min" value: (empty(autonomous_car) ? 0.0 : min(autonomous_car collect each.speed)) color: #indigo;
            }
        }
        display "Throughput" type: 2d refresh: every(2 #cycles) {
            chart "Traffic Throughput" type: series {
                data "Trips Completed" value: avs_completed color: #purple;
                data "Avg Travel Time" value: (avs_completed > 0 ? total_av_travel_time / avs_completed : 0.0) color: #orange;
                data "Active Vehicles" value: length(autonomous_car) color: #green;
            }
        }
    }
}

experiment MixedTraffic type: gui {
    init { 
        spawn_regular_cars <- true; 
        spawn_autonomous_cars <- true;
        nb_autonomous_cars <- nb_cars;  // Keep equal for fair comparison
    }
    parameter "Vehicles per Type" var: nb_cars min: 1 max: 75;
    
    output {
        display "Mixed Traffic" background: #white type: 2d axes: false autosave: false {
            species road; species intersection; species traffic_light; species spawn_node; species car; species autonomous_car;
        }
        display "Speed Comparison" type: 2d refresh: every(2 #cycles) {
            chart "Human vs AV Speed" type: series {
                data "Human Avg" value: (empty(car) ? 0.0 : mean(car collect each.speed)) color: #blue;
                data "AV Avg" value: (empty(autonomous_car) ? 0.0 : mean(autonomous_car collect each.speed)) color: #purple;
            }
        }
        display "Throughput Comparison" type: 2d refresh: every(2 #cycles) {
            chart "Trips Completed" type: series {
                data "Human Trips" value: cars_completed color: #blue;
                data "AV Trips" value: avs_completed color: #purple;
            }
        }
        display "Travel Time" type: 2d refresh: every(2 #cycles) {
            chart "Average Travel Time (Lower = Better)" type: series {
                data "Human Avg Time" value: (cars_completed > 0 ? total_car_travel_time / cars_completed : 0.0) color: #blue;
                data "AV Avg Time" value: (avs_completed > 0 ? total_av_travel_time / avs_completed : 0.0) color: #purple;
            }
        }
    }
}
