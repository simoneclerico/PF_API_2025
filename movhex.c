#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define HASH_MAP_EMPTY ((uint16_t)65535)
#define HASH_MAP_TOMBSTONE ((uint16_t)65534)
#define POS_FAIL ((dim_t)~(dim_t)0)
#define CACHE_EMPTY ((dim_t)4294967295)
#define CACHE_TOMBSTONE ((dim_t)4294967294)

typedef uint32_t coord_t;
typedef uint32_t dim_t;

/*
 * Each hex on the grid stores only its exit cost (0 = impassable, 1-100 = passable).
 * The grid is a flat array; (x, y) maps to index y*g_cols + x.
 */
typedef struct{
    uint8_t cost;
}hex;

/* Cube coordinate representation used for hex-distance calculations in change_cost. */
typedef struct{
    int32_t  q, r, s;
}cube;

typedef struct{
    coord_t x, y;
}offsetCoord;


/*
 * Stores all outgoing air routes for a single hex.
 * At most 5 routes per hex (spec constraint), so fixed-size arrays suffice.
 * grid_idx is the flat-array index of the source hex.
 */
typedef struct{
    dim_t dest[5];
    uint8_t cost[5];
    uint8_t cont;
    dim_t grid_idx;
}airHex;

/*
 * A heap entry carries a timestamp (ts) matching the tc_execution_cont value of the
 * Dijkstra call that pushed it. Stale entries (wrong ts) are silently discarded on pop,
 * making the heap reusable across calls without ever clearing it.
 */
typedef struct{
    dim_t node;
    dim_t cost;
    dim_t ts;
}heap_hex;

/* Cache entry for a (start, dest) pair: stores the travel_cost result and the
 * cache_version at insertion time. Entries from older versions are ignored. */
typedef struct{
    dim_t start;
    dim_t dest;
    int32_t value;
    uint32_t version;
}cache_hex;

/*
 * Hex adjacency offsets in offset (odd-r) coordinates.
 * In this layout odd-numbered rows are shifted half a hex to the right,
 * so the six neighbor deltas differ depending on row parity.
 * Each entry is {delta_col, delta_row}.
 */
const int vicini_righe_pari[6][2] = { {+1, 0}, {0, +1}, {-1, +1}, {-1, 0}, {-1, -1}, {0, -1} };

const int vicini_righe_dispari[6][2] = { {+1, 0}, {+1, +1}, {0, +1}, {-1, 0}, {0, -1}, {+1, -1} };


coord_t g_cols = 0, g_rows = 0;

hex* grid = NULL;

dim_t hwr_capacity = 0;   // hwr = hex with routes
dim_t hwr_cont = 0;
airHex* hex_with_routes = NULL;
uint16_t* routes_hash_map = NULL;    // hash map size is 2x capacity to keep load factor low

dim_t tc_execution_cont;    // tc = Travel Cost
dim_t* dist = NULL;
dim_t* ts_dist = NULL;
uint8_t* visited = NULL;
dim_t* ts_visited = NULL;

heap_hex* priority_queue = NULL;
dim_t heap_cont = 0;
dim_t heap_capacity = 0;

dim_t cache_version = 0;
cache_hex* cache = NULL;
dim_t cache_size = 0;
dim_t cache_cont = 0;
dim_t cache_mask = 0;


// FUNCTION DECLARATIONS

// init

void free_all();

int init(coord_t, coord_t);

dim_t index_calculator(coord_t, coord_t);


// change_cost

bool in_bounds(coord_t, coord_t);

cube oddr_to_cube(coord_t, coord_t);

offsetCoord cube_to_oddr(int32_t, int32_t, int32_t);

int change_cost(coord_t, coord_t, int8_t, coord_t );


// toggle_air_route

void init_air_routes();

dim_t pos_lookup(dim_t);

dim_t pos_insert(dim_t);

void riallocazione();

uint8_t route_cost_calculator(dim_t, dim_t);

bool find_route(dim_t, dim_t, dim_t);

int toggle_air_route(coord_t, coord_t, coord_t, coord_t);


// travel_cost

void heap_push(dim_t, dim_t, dim_t);

int heap_pop(dim_t*, dim_t*, dim_t);

void init_tc();

int travel_cost(coord_t, coord_t, coord_t, coord_t);

void init_cache();

dim_t hash_function_cache(dim_t, dim_t);

void rehash_cache();

bool cache_get(dim_t, dim_t, int*);

void cache_insert(dim_t, dim_t, int);


// stampa

void stampa();

void stampa_rotte_aeree();


// MAIN

int main(){

    char command_string[50];

    coord_t x1 = 0;
    coord_t y1 = 0;
    coord_t x2 = 0;
    coord_t y2 = 0;
    int8_t value = 0;
    coord_t radius = 0;

    int result = 0;

    while(scanf("%49s", command_string) == 1){

        if(strcmp(command_string, "init") == 0){

            if(scanf(" %u %u", &x1, &y1) == 2){

                result = init(x1, y1);

                if(result) printf("OK\n");
                else printf("KO\n");

            }

            else exit(EXIT_FAILURE);

        }

        else if(strcmp(command_string, "change_cost") == 0){

            if(scanf(" %u %u %hhd %u", &x1, &y1, &value, &radius) == 4){

                result = change_cost(x1, y1, value, radius);

                if(result) printf("OK\n");
                else printf("KO\n");

            }

            else exit(EXIT_FAILURE);

        }

        else if(strcmp(command_string, "toggle_air_route") == 0){

            if(scanf(" %u %u %u %u", &x1, &y1, &x2, &y2) == 4){

                result = toggle_air_route(x1, y1, x2, y2);

                if(result) printf("OK\n");
                else printf("KO\n");

            }

            else exit(EXIT_FAILURE);
        }

        else{

            if(scanf(" %u %u %u %u", &x1, &y1, &x2, &y2) == 4){

                result = travel_cost(x1, y1, x2, y2);

                printf("%d\n", result);

            }

            else exit(EXIT_FAILURE);
        }

    }

    free_all();

    return 0;
}


// FUNCTIONS

int init(coord_t cols, coord_t rows){

    free_all();

    cache_version++;

    if(cache_version == 0){

        memset(cache, 0xFF, cache_size * sizeof(cache_hex));
        cache_cont = 0;
        cache_version = 1;

    }

    g_cols = cols;
    g_rows = rows;
    dim_t N = (dim_t)g_cols * (dim_t)g_rows;

    grid = malloc(N * sizeof(hex));

    init_air_routes();
    init_tc();
    init_cache();

    if(!grid || !hex_with_routes || !routes_hash_map || !dist || !ts_dist || !visited || !ts_visited || !priority_queue || !cache) exit(EXIT_FAILURE);

    memset(grid, 1, N * sizeof(hex));

    return 1;
}

void free_all(){

    if(grid){
        free(grid);
        grid = NULL;
    }

    if(hex_with_routes){
        free(hex_with_routes);
        hex_with_routes = NULL;
    }

    if(routes_hash_map){
        free(routes_hash_map);
        routes_hash_map = NULL;
    }

    if(dist){
        free(dist);
        dist = NULL;
    }

    if(ts_dist){
        free(ts_dist);
        ts_dist = NULL;
    }

    if(visited){
        free(visited);
        visited = NULL;
    }

    if(ts_visited){
        free(ts_visited);
        ts_visited = NULL;
    }

    if(priority_queue){
        free(priority_queue);
        priority_queue = NULL;
    }

    if(cache){
        free(cache);
        cache = NULL;
    }

}

dim_t index_calculator(coord_t x, coord_t y){

    dim_t i = y * g_cols + x;

    return i;

}

bool in_bounds(coord_t x, coord_t y){

    return (x < g_cols && y < g_rows);
}

// converts offset (odd-r) coordinates to cube coordinates
cube oddr_to_cube(coord_t x, coord_t y){

    int parity = y&1;
    int q_ax = x - (y - parity) / 2;
    int r_ax = y;

    return (cube){.q = q_ax, .r = r_ax, .s = -q_ax -r_ax};
}

offsetCoord cube_to_oddr(int32_t q, int32_t r, int32_t s){

    int parity = r&1;
    int32_t x_oddr = q + (r - parity) / 2;
    int32_t y_oddr = r;

    return (offsetCoord){.x = (coord_t)x_oddr, .y = (coord_t)y_oddr};
}

int change_cost(coord_t xc, coord_t yc, int8_t v, coord_t r){
    if(!in_bounds(xc, yc) || v < -10 || v > 10 || r <= 0) return 0;

    cache_version++;

    // cache_version acts as a generation counter: entries from a previous version are
    // treated as invalid without needing to clear the entire cache table
    if(cache_version == 0){

        memset(cache, 0xFF, cache_size * sizeof(cache_hex));
        cache_cont = 0;
        cache_version = 1;

    }

    int R = (int)r;

    cube center = oddr_to_cube(xc, yc);

	/*
     * Enumerate all hexes within hex-distance < R from the centre by iterating
     * over cube coordinates (dq, dr, ds with dq+dr+ds=0). The ranges
     * dr in [max(-R,-dq-R), min(R,-dq+R)] guarantee ds = -dq-dr stays in [-R,R].
     */
    for(int dq = -R ; dq <= R ; dq++){

        for(int dr = MAX(-R, -dq -R ) ; dr <= MIN(R, -dq + R) ; dr++){

            int ds = - dq - dr;

            cube c = {.q = center.q + dq, .r = center.r + dr , .s = center.s + ds};
            offsetCoord h = cube_to_oddr(c.q, c.r, c.s);

            if(!in_bounds(h.x, h.y)) continue;

            dim_t i = index_calculator(h.x, h.y);
            int hex_dist = (abs(dq) + abs(dr) + abs(ds)) / 2;

            int t = R - hex_dist;    // t > 0 for hexes strictly inside the radius

            if(t <= 0) continue;	// boundary ring (distance == R) is excluded per spec

            int delta = 0;

            int64_t num = (int64_t)v * (int64_t)t;

			// floor division that rounds toward -inf for negative values
            if(num >= 0) delta = (int)(num / R);

            else delta = -(int)((-num + R -1) / R);

            int sum = (int)grid[i].cost + delta;

            // clamp to [0, 100] as required by the spec
            if(sum < 0) sum = 0;
            if(sum > 100) sum = 100;

            grid[i].cost = (uint8_t)sum;

            dim_t pos = 0;
            pos = pos_lookup(i);

            if(pos != POS_FAIL){

                dim_t hash_idx = routes_hash_map[pos];

                if(hash_idx < hwr_cont && hex_with_routes[hash_idx].grid_idx == i){

                    airHex* ah = &hex_with_routes[hash_idx];

                    for(int j = 0 ; j < ah->cont ; j++){
                        // air route costs are stored per-route and must be kept in sync
                        // with the source hex cost whenever change_cost modifies it
                        ah->cost[j] = sum;
                    }

                }

            }

        }
    }

    return 1;
}

void init_air_routes(){

    hwr_capacity = 1024;
    hwr_cont = 0;

    hex_with_routes = malloc(hwr_capacity * sizeof(airHex));
    routes_hash_map = malloc(2 * hwr_capacity * sizeof(uint16_t));

    if(!hex_with_routes || !routes_hash_map) exit(EXIT_FAILURE);

    // all slots initialised to EMPTY (0xFFFF); valid hex_with_routes indices must stay in [0, 65533]
    memset(routes_hash_map, 0xFF, 2 * hwr_capacity * sizeof(uint16_t));

}

/*
 * pos_lookup / pos_insert use the flat grid index directly as the hash key
 * (key & mask). This is an identity hash that distributes well when hexes
 * with air routes are scattered across the grid, which is the common case.
 * The table size is always 2*hwr_capacity, keeping load factor <= 0.5.
 */
dim_t pos_lookup(dim_t key){

    if(hwr_capacity == 0) return POS_FAIL;

    dim_t mask = (2 * hwr_capacity - 1);

    dim_t curr = key & mask;

    for(dim_t i = 0 ; i < mask + 1 ; i++){

        dim_t pos = (curr + i) & mask;

        dim_t hash_idx = routes_hash_map[pos];

        if(hash_idx == HASH_MAP_EMPTY) return POS_FAIL;

        if(hash_idx != HASH_MAP_TOMBSTONE && hash_idx < hwr_cont){

            airHex* ah = &hex_with_routes[hash_idx];
            if(ah->grid_idx == key) return pos;

        }

    }

    return POS_FAIL;

}

dim_t pos_insert(dim_t key){

    if(hwr_capacity == 0) return POS_FAIL;

    dim_t mask = (2 * hwr_capacity - 1);

    dim_t curr = key & mask;

    dim_t first_tombstone = POS_FAIL;

    for(dim_t i = 0 ; i < mask + 1 ; i++){

        dim_t pos = (curr + i) & mask;

        dim_t hash_idx = routes_hash_map[pos];

        if(hash_idx == HASH_MAP_EMPTY) return (first_tombstone != POS_FAIL ? first_tombstone : pos);

        if(hash_idx == HASH_MAP_TOMBSTONE){

            if(first_tombstone == POS_FAIL) first_tombstone = pos;

            continue;

        }

        airHex* ah = &hex_with_routes[hash_idx];

        if(hash_idx < hwr_cont && ah->grid_idx == key) return pos;

    }

    return first_tombstone != POS_FAIL ? first_tombstone : POS_FAIL;

}

/*
 * Doubles hwr_capacity and rebuilds the hash map in-place.
 * The new upper half of routes_hash_map is zeroed to EMPTY first.
 * Then each valid entry in the old half is cleared and reinserted so
 * it lands in its correct slot under the new (larger) mask.
 */
void riallocazione(){

    dim_t old_capacity = hwr_capacity;
    hwr_capacity *= 2;

    airHex* temp1 =  realloc(hex_with_routes, hwr_capacity * sizeof(airHex));
    uint16_t* temp2 = realloc(routes_hash_map, 2 * hwr_capacity * sizeof(uint16_t));

    if(!temp1 || !temp2 ) exit(EXIT_FAILURE);

    hex_with_routes = temp1;
    routes_hash_map = temp2;

    memset(routes_hash_map + (2 * old_capacity), 0xFF, (2 * hwr_capacity - 2 * old_capacity) * sizeof(uint16_t));

    for(dim_t i = 0 ; i < 2 * old_capacity ; i++){

        dim_t idx = routes_hash_map[i];

        if(idx == HASH_MAP_EMPTY || idx == HASH_MAP_TOMBSTONE) continue;

        routes_hash_map[i] = HASH_MAP_EMPTY;

        dim_t new_rehash_pos = pos_insert(hex_with_routes[idx].grid_idx);

        if(new_rehash_pos == POS_FAIL) exit(EXIT_FAILURE);

        routes_hash_map[new_rehash_pos] = idx;

    }

}

/*
 * Computes the cost of a new air route leaving `index`.
 * Per spec: floor((sum_of_existing_air_route_costs + hex_exit_cost) / (num_routes + 1)).
 * If the hex has no routes yet, the cost equals the hex exit cost.
 */
uint8_t route_cost_calculator(dim_t index, dim_t pos){

    if(pos == POS_FAIL) return grid[index].cost;

    dim_t hash_idx = routes_hash_map[pos];
    airHex* ah = &hex_with_routes[hash_idx];

    if(hash_idx == HASH_MAP_EMPTY || hash_idx == HASH_MAP_TOMBSTONE || hash_idx >= hwr_cont) return grid[index].cost;

    if(ah->grid_idx != index) return grid[index].cost;

    dim_t sum = 0;
    dim_t cont = 0;

    for(int i = 0 ; i < ah->cont ; i++){
        sum += ah->cost[i];
        cont++;
    }

    return (uint8_t)((sum + grid[index].cost) / (cont + 1));
}

bool find_route(dim_t id1, dim_t id2, dim_t pos){

    if(pos == POS_FAIL) return false;

    dim_t hash_idx = routes_hash_map[pos];
    airHex* ah = &hex_with_routes[hash_idx];

    if(hash_idx == HASH_MAP_EMPTY || hash_idx == HASH_MAP_TOMBSTONE || hash_idx >= hwr_cont) return false;

    if(ah->grid_idx != id1) return false;

    for(int i = 0 ; i < ah->cont ; i++){
        if(ah->dest[i] == id2) return true;
    }

    return false;
}


int toggle_air_route(coord_t x1, coord_t y1, coord_t x2, coord_t y2){

    if(!in_bounds(x1, y1) || !in_bounds(x2, y2)) return 0;

    if(hwr_cont == hwr_capacity) riallocazione();

    dim_t id1 = 0;
    dim_t id2 = 0;
    dim_t pos = 0;

    id1 = index_calculator(x1, y1);
    id2 = index_calculator(x2, y2);
    pos = pos_lookup(id1);

    if(pos == POS_FAIL){     // starting hex has no routes (not yet added to hex_with_routes)

        cache_version++;

        if(cache_version == 0){

            memset(cache, 0xFF, cache_size * sizeof(cache_hex));
            cache_cont = 0;
            cache_version = 1;

        }

        pos = pos_insert(id1);

        if(pos == POS_FAIL) exit(EXIT_FAILURE);

        if(hwr_cont == hwr_capacity) riallocazione();

        hex_with_routes[hwr_cont].cont = 0;
        hex_with_routes[hwr_cont].grid_idx = id1;

        for (int i = 0; i < 5; i++) {

            hex_with_routes[hwr_cont].dest[i] = 65535;
            hex_with_routes[hwr_cont].cost[i] = 0;

        }

        hex_with_routes[hwr_cont].dest[hex_with_routes[hwr_cont].cont] = id2;
        hex_with_routes[hwr_cont].cost[hex_with_routes[hwr_cont].cont] = route_cost_calculator(id1, pos);
        hex_with_routes[hwr_cont].cont++;

        routes_hash_map[pos] = (uint16_t)hwr_cont;
        hwr_cont++;

    }

    else if(find_route(id1, id2, pos)){      // starting hex has routes and has that specific route

        cache_version++;

        if(cache_version == 0){

            memset(cache, 0xFF, cache_size * sizeof(cache_hex));
            cache_cont = 0;
            cache_version = 1;

        }

		// swap-and-pop: replace the removed route with the last one to avoid shifting
        int found = -1;

        dim_t hash_idx = routes_hash_map[pos];
        airHex* ah = &hex_with_routes[hash_idx];

        for(int i = 0 ; i < ah->cont ; i++){

            if(ah->dest[i] == id2){

                found = i;
                break;
            }

        }

        if(found >= 0){

            int last = (int)ah->cont - 1;

            if(found != last){
                ah->dest[found] = ah->dest[last];
                ah->cost[found] = ah->cost[last];
            }

            ah->dest[last] = 65535;
            ah->cost[last] = 0;
            ah->cont--;

        }

        // if no routes remain, remove the entry from hex_with_routes without leaving gaps
        if(ah->cont == 0){

            dim_t temp_idx = hash_idx;

            routes_hash_map[pos] = HASH_MAP_TOMBSTONE;

            if(temp_idx != hwr_cont - 1){

                hex_with_routes[temp_idx] = hex_with_routes[hwr_cont - 1];

                dim_t moved_key = hex_with_routes[temp_idx].grid_idx;

                dim_t old_pos = pos_lookup(moved_key);

                if(old_pos == POS_FAIL) exit(EXIT_FAILURE);

                routes_hash_map[old_pos] = HASH_MAP_TOMBSTONE;

                dim_t new_pos = pos_insert(moved_key);

                if(new_pos == POS_FAIL) exit(EXIT_FAILURE);

                routes_hash_map[new_pos] = (uint16_t)temp_idx;

            }

            hwr_cont--;

        }

    }

    else if(hex_with_routes[routes_hash_map[pos]].cont >= 5) return 0;    // starting hex has routes but does not have that specific route, yet the count is greater than or equal to 5

    else{        // starting hex has routes but does not have that specific route, and the count is less than 5

        cache_version++;

        if(cache_version == 0){

            memset(cache, 0xFF, cache_size * sizeof(cache_hex));
            cache_cont = 0;
            cache_version = 1;

        }

        dim_t hash_idx = routes_hash_map[pos];
        airHex* ah = &hex_with_routes[hash_idx];

        ah->dest[ah->cont] = id2;
        ah->cost[ah->cont] = route_cost_calculator(id1, pos);
        ah->cont++;

    }

    return 1;

}

void heap_push(dim_t node, dim_t cost, dim_t curr){

    if(heap_cont == heap_capacity){

        heap_capacity *= 2;
        priority_queue = realloc(priority_queue, heap_capacity * sizeof(heap_hex));

    }

    dim_t i = heap_cont++;
    priority_queue[i].node = node;
    priority_queue[i].cost = cost;
    priority_queue[i].ts = curr;

    // heapify-up
    while(i > 0){

        dim_t padre = (i - 1) / 2;
        if(priority_queue[padre].cost <= priority_queue[i].cost) break;

        heap_hex temp = priority_queue[padre];
        priority_queue[padre] = priority_queue[i];
        priority_queue[i] = temp;

        i = padre;

    }

}

int heap_pop(dim_t* node, dim_t* cost, dim_t ts_curr){

    while(heap_cont > 0){

        heap_hex min = priority_queue[0];
        priority_queue[0] = priority_queue[--heap_cont];

        // heapify-down
        dim_t i = 0;

        while(1){

            dim_t left = 2 * i + 1;
            dim_t right = 2 * i + 2;
            dim_t smallest = i;

            if(left < heap_cont && priority_queue[left].cost < priority_queue[smallest].cost) smallest = left;

            if(right < heap_cont && priority_queue[right].cost < priority_queue[smallest].cost) smallest = right;

            if(smallest ==  i) break;

            heap_hex temp = priority_queue[i];
            priority_queue[i] = priority_queue[smallest];
            priority_queue[smallest] = temp;

            i = smallest;

        }

        if(min.ts != ts_curr) continue;

        *node = min.node;
        *cost = min.cost;

        return 1;

    }

    return 0;   // priority queue exhausted, no path found

}

void init_tc(){

    dim_t N = g_cols * g_rows;

    dist = malloc(N * sizeof(dim_t));
    ts_dist = malloc(N * sizeof(dim_t));
    visited = malloc(N * sizeof(uint8_t));
    ts_visited = malloc(N * sizeof(dim_t));

    // timestamps allow O(1) logical reset: incrementing tc_execution_cont invalidates all
    // previous dist/visited entries without touching the arrays
    // timestamps allow O(1) logical reset per Dijkstra call: incrementing tc_execution_cont
    // invalidates all previous dist/visited entries without zeroing the arrays each time
    memset(ts_dist, 0, N * sizeof(dim_t));
    memset(ts_visited, 0, N * sizeof(dim_t));

    heap_cont = 0;
    heap_capacity = 16384;

    tc_execution_cont = 0;

    priority_queue = malloc(heap_capacity * sizeof(heap_hex));

}

int travel_cost(coord_t x1, coord_t y1, coord_t x2, coord_t y2){

    if(!in_bounds(x1, y1) || !in_bounds(x2, y2)) return -1;

    dim_t id1 = 0;
    dim_t id2 = 0;

    id1 = index_calculator(x1, y1);
    id2 = index_calculator(x2, y2);

    int cached;
    if(cache_get(id1, id2, &cached)) return cached;

    // trivial case: source equals destination
    if(id1 == id2){

        cache_insert(id1, id2, 0);
        return 0;

    }

    heap_cont = 0;

    tc_execution_cont++;

    // if the timestamp counter wraps around, reset ts arrays to avoid stale matches (extremely rare)
    if(tc_execution_cont == 0){

        tc_execution_cont = 1;

        memset(ts_dist, 0, g_cols * g_rows * sizeof(dim_t));
        memset(ts_visited, 0, g_cols * g_rows * sizeof(dim_t));

    }

    // starting hex initialization
    dist[id1] = 0;
    ts_dist[id1] = tc_execution_cont;
    visited[id1] = 0;
    ts_visited[id1] = tc_execution_cont;

    heap_push(id1, 0, tc_execution_cont);

    dim_t u = 0;
    dim_t popped_node = 0;
    dim_t popped_cost = 0;

    while(heap_pop(&popped_node, &popped_cost, tc_execution_cont)){

        u = popped_node;
        dist[u] = popped_cost;

        if(ts_visited[u] == tc_execution_cont && visited[u]) continue;

        visited[u] = 1;
        ts_visited[u] = tc_execution_cont;

        // early exit once the destination node is settled
        if(u == id2){

            cache_insert(id1, id2, (int)dist[id2]);
            return dist[id2];
        }

        // impassable hex: do not expand neighbors (cost 0 = cannot leave)
        if(grid[u].cost == 0) continue;

        dim_t curr_col = u % (dim_t)g_cols;
        dim_t curr_row = u / (dim_t)g_cols;

        const int (*offsets)[2] = (curr_row & 1) ? vicini_righe_dispari : vicini_righe_pari;

        dim_t new_cost = dist[u] + grid[u].cost;

        for(int k = 0 ; k < 6 ; k++){

            int new_col = curr_col + offsets[k][0];
            int new_row = curr_row + offsets[k][1];

            if((unsigned)new_col >= g_cols || (unsigned)new_row >= g_rows) continue;

            dim_t v = index_calculator(new_col, new_row);

            if(grid[v].cost == 0 && v != id2) continue;

            if(ts_dist[v] != tc_execution_cont || new_cost < dist[v]){

                dist[v] = new_cost;
                ts_dist[v] = tc_execution_cont;
                visited[v] = 0;
                ts_visited[v] = tc_execution_cont;
                heap_push(v, new_cost, tc_execution_cont);
            }

        }

        // expand air routes departing from the current hex
        dim_t pos = pos_lookup(u);

        if(pos != POS_FAIL){

            dim_t idx = routes_hash_map[pos];

            if(idx < hwr_cont && hex_with_routes[idx].grid_idx == u){

                airHex* ah = &hex_with_routes[idx];

                for(int r = 0 ; r < ah->cont ; r++){

                    dim_t v = ah->dest[r];

                    // skip if the route destination is impassable (unless it is the travel_cost target)
                    if(grid[v].cost == 0 && v != id2) continue;

                    if(ts_dist[v] != tc_execution_cont || new_cost < dist[v]){

                        dist[v] = new_cost;
                        ts_dist[v] = tc_execution_cont;
                        visited[v] = 0;
                        ts_visited[v] = tc_execution_cont;
                        heap_push(v, new_cost, tc_execution_cont);

                    }

                }

            }

        }

    }

    // destination unreachable (or source hex has cost 0)
    cache_insert(id1, id2, -1);
    return -1;

}

void init_cache(){

    cache_version = 1;
    cache_cont = 0;
    cache_size = 65536;
    cache_mask = cache_size - 1;

    cache = malloc(cache_size * sizeof(cache_hex));

    if(!cache) exit(EXIT_FAILURE);

    memset(cache, 0xFF, cache_size * sizeof(cache_hex));

}

dim_t hash_function_cache(dim_t id1, dim_t id2){

    // Fibonacci hashing: multiply by the golden-ratio-derived 64-bit constant for good avalanche
    const uint64_t a = 11400714819323198485ull;

    uint64_t k = ((uint64_t)id1 << 32) | (uint64_t)id2;
    uint64_t h = k * a;

    return (dim_t)h & cache_mask;

}

void rehash_cache(){

    cache_cont = 0;
    dim_t old_size = cache_size;
    cache_size *= 2;
    cache_mask = cache_size - 1;

    cache_hex* cache_temp = realloc(cache, cache_size * sizeof(cache_hex));

    if(!cache_temp) exit(EXIT_FAILURE);

    cache = cache_temp;
    memset(cache + old_size, 0xFF, (cache_size - old_size) * sizeof(cache_hex));

    for(int i = 0 ; i < old_size ; i++){

        if(cache[i].start != CACHE_EMPTY && cache[i].start != CACHE_TOMBSTONE && cache[i].version == cache_version){

            dim_t h = hash_function_cache((uint64_t)cache[i].start, (uint64_t)cache[i].dest);

            while(cache[h].start != CACHE_EMPTY){

                h = (h + 1) & cache_mask;

            }

            if(h != i){

                cache_hex temp = cache[i];
                cache[i].start = CACHE_TOMBSTONE;
                cache[h] = temp;

            }

            cache_cont++;

        }

    }

}

bool cache_get(dim_t id1, dim_t id2, int* value){

    if(!cache) return false;

    dim_t h = hash_function_cache(id1, id2);

    while(1){

        if(cache[h].start == CACHE_EMPTY) return false;

        if(cache[h].start != CACHE_TOMBSTONE && cache[h].start == id1 && cache[h].dest == id2){

            if(cache[h].version == cache_version){

                *value = cache[h].value;
                return true;

            }

            else return false;

        }

        h = (h + 1) & cache_mask;

    }

}

void cache_insert(dim_t id1, dim_t id2, int value){

    // keep load factor <= 0.7 before inserting
    if((cache_cont + 1) * 10 > cache_size * 7) rehash_cache();

    dim_t h = hash_function_cache(id1, id2);
    int32_t first_tombstone = -1;

    while(1){

        if(cache[h].start == CACHE_EMPTY) break;

        if(cache[h].start == CACHE_TOMBSTONE && first_tombstone < 0) first_tombstone = h;

        if(cache[h].start != CACHE_TOMBSTONE && cache[h].start == id1 && cache[h].dest == id2){

            cache[h].value = (int32_t)value;
            cache[h].version = cache_version;
            return;

        }

        h = (h + 1) & cache_mask;

    }

    dim_t slot_found = (first_tombstone >= 0) ? (dim_t)first_tombstone : h;

    cache[slot_found].start = id1;
    cache[slot_found].dest = id2;
    cache[slot_found].value = value;
    cache[slot_found].version = cache_version;

    cache_cont++;

}
