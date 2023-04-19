#include <iostream>
#include <vector>
#include <cstring>
#include <set>
#include <map>
#include <algorithm>
#include <chrono>
#include <signal.h>

#include <utils/ipv4_utils.h>
#include <utils/cl_option.h>

namespace sdf_fsm {

	typedef void(*on_find_action) (void* arg);

	struct alignas(32) value_range {
		uint32_t range_start = 0;
		uint32_t range_end = 0;
		
		void print() {
			std::cout << "range start: " << range_start << ", as: " << ipv4_utils::itos(range_start) << std::endl;
			std::cout << "range end: " << range_end << ", as: " << ipv4_utils::itos(range_end) << std::endl;
		}

		bool operator==(value_range& other) {
			return (other.range_start == range_start) and (other.range_end == range_end);
		}

		bool operator==(uint32_t value) {
			return ((range_start <= value) and (range_end >= value));
		}
	};

	struct alignas(32) match_params {
		uint32_t proto = 0;
		uint32_t source_ip = 0;
		uint32_t source_port = 0;
		uint32_t dest_ip = 0;
		uint32_t dest_port = 0;
		void print() {
			std::cout << proto 
				<< " from " << ipv4_utils::itos(source_ip) << " " << source_port
				<< " to " << ipv4_utils::itos(dest_ip) << " " << dest_port
				<<std::endl;
		}
	};

	struct alignas(32) filter_params {
		value_range proto;

		value_range source_ip;
		value_range source_port;

		value_range dest_ip;
		value_range dest_port;

		uint32_t reserved = 0;

		void print_params() {
			std::cout << "sdf filter params: " << std::endl;
			
			std::cout << "\tprotocol: from " << std::to_string(proto.range_start) << ", to: " << std::to_string(proto.range_end) << std::endl;
			std::cout << "\tsource ip from: " << std::to_string(source_ip.range_start) << ", to: " << std::to_string(source_ip.range_end) << std::endl;
			std::cout << "\tsource port from: " << std::to_string(source_port.range_start) << ", to: " << std::to_string(source_port.range_end) << std::endl;
			std::cout << "\tdest ip from: " << std::to_string(dest_ip.range_start) << ", to: " << std::to_string(dest_ip.range_end) << std::endl;
			std::cout << "\tdest port from: " << std::to_string(dest_port.range_start) << ", to: " << std::to_string(dest_port.range_end) << std::endl;
			
		}
	};

	struct state;

	struct non_deterministic_transition {
		std::set<state*> states {
			nullptr
		};
	};
	
	struct state {
		std::string name = "none";
		std::map<uint32_t, non_deterministic_transition> transitions {
			{0, non_deterministic_transition()}
		};
		std::vector<state*> state_container;
		bool is_final = false;
		void* user_data = nullptr;
	};

	state* init_root() {
		return new state();
	}

	static size_t state_id = 0;

	int add_filter(state* root, filter_params* params) {
		value_range* param_it = (value_range*)params;
		state* current_state = root;
		while (param_it != (value_range*)&(params->reserved)) {
			state* new_state = new state();
			root->state_container.push_back(new_state);
			new_state->name = std::to_string (state_id++);
			auto l_boundary_range = current_state->transitions.lower_bound(param_it->range_start);
			if(l_boundary_range != current_state->transitions.begin()) {
				--l_boundary_range;
			}
			non_deterministic_transition* current_transition = &(l_boundary_range->second);
			if(l_boundary_range->first != param_it->range_start) {
				current_transition = new non_deterministic_transition();
				current_transition->states = std::set<state*>(l_boundary_range->second.states.begin(), l_boundary_range->second.states.end());
			}

			auto r_boundary_range = current_state->transitions.lower_bound((param_it->range_end + 1));
			if(r_boundary_range != current_state->transitions.begin()) {
				--r_boundary_range;
			}
			while (l_boundary_range != r_boundary_range) {
				l_boundary_range->second.states.insert(new_state);
				++l_boundary_range;
			}
			current_state->transitions.insert({
				param_it->range_end + 1,
				non_deterministic_transition {
					.states = std::set<state*>(l_boundary_range->second.states.begin(), l_boundary_range->second.states.end())
				}
			});

			current_transition->states.insert(new_state); //TODO allocate new state
			current_state->transitions.insert({
				param_it->range_start,
				*current_transition
			});
			current_state = new_state; // TODO allocate new state
			++param_it;
		}
		return 0;
	}

	int test_sdf_fsm() {
		
		state* root = init_root();
		int rc = 0;
		filter_params test_prm {
			.proto = {2, 6}
		};
		
		rc = add_filter(root, &test_prm);
		test_prm.proto = {7,11};
		rc = add_filter(root, &test_prm);
		test_prm.proto = {4,6};
		rc = add_filter(root, &test_prm);
		test_prm.proto = {7, 9};
		rc = add_filter(root, &test_prm);
		test_prm.proto = {3, 5};
		rc = add_filter(root, &test_prm);
		test_prm.proto = {6, 10};
		return 0;
	}

}// sdf_fsm

namespace sdf_filter {
	
	typedef int (*apply_action_t)(char* l_arg, char* r_arg, void* result);

	struct step {
		apply_action_t action = nullptr;
		char* r_arg = nullptr;
		void* res = nullptr;
		int try_step(char* l_arg) {
			return action(l_arg, r_arg, res);
		}
	};

	int verify(char* l_arg, char* r_arg, void* result) {
		int rc = strcmp(l_arg, (char*)r_arg);
		*((bool*)result) = true;
		if(rc != 0) *((bool*)result) = false;
		return 0;
	}
	
	int get_proto(char* l_arg, char*, void* filled_data) {
		((sdf_fsm::value_range*)filled_data)->range_start = atoi(l_arg);
		((sdf_fsm::value_range*)filled_data)->range_end = ((sdf_fsm::value_range*)filled_data)->range_start;
		return 0;
	}

	int get_port_range(char* l_arg, char*, void* filled_data) {
		char* pos_saver;
		char* pos = strtok_r(l_arg, "-", &pos_saver);
		uint32_t* fill_pos = (uint32_t*)filled_data;
		size_t n_args = 0;
		while(pos != nullptr) {
			fill_pos[n_args] = atoi(pos);
			if(++n_args == 2) {
				break;
			}
			pos = strtok_r(NULL, "-", &pos_saver);
		}
		if(n_args < 1) {
			std::cerr << "not enough arguments to parse port range" << std::endl;
			return -1;
		}
		if(n_args == 1) {
			fill_pos[n_args] = fill_pos[n_args - 1];
		}
		return 0;
	}

	int get_addr_range(char* l_arg, char*, void* filled_data) {
		sdf_fsm::value_range* range = (sdf_fsm::value_range*)filled_data;
		//parse source ip addr
		uint32_t ip_addr;
		uint32_t mask_as_ipv4;
		int rc = ipv4_utils::stoi_more(l_arg, &ip_addr, &mask_as_ipv4);
		if(rc != 0) {
			std::cerr << "error while ipv4_utils::stoi_more()" << std::endl;
			return -1;
		}
		range->range_start = (ip_addr & mask_as_ipv4);
		range->range_end = (range->range_start | (~mask_as_ipv4));
		return 0;
	}

	int parse_filter_params(char* filter_string, sdf_fsm::filter_params* filled_params) {
		bool success = true;
		int rc  = 0;

		step roadmap[] = {
			{verify, "permit", &success},
			{verify, "out", &success},
			{get_proto, nullptr, &(filled_params->proto)},
			{verify, "from", &success},
			{get_addr_range, nullptr, &(filled_params->source_ip)},
			{get_port_range, nullptr, &(filled_params->source_port)},
			{verify, "to", &success},
			{get_addr_range, nullptr, &(filled_params->dest_ip)},
			{get_port_range, nullptr, &(filled_params->dest_port)},
		};
		char delim[] = {
			' ',
		};

		char* split_str = strtok(filter_string, delim);
		for(auto& _step: roadmap) {
			if(split_str == nullptr) {
				std::cerr << "error: not enough params in filter" << std::endl;
				return -1;
			}
			rc  = _step.try_step(split_str);
			if(rc != 0) {
				std::cerr << "error while step::try_step()" << std::endl;
				return -1;
			}

			if(not success) {
				std::cerr << "verification of " << split_str << " failed" << std::endl;
				return -1;
			}
			split_str = strtok(NULL, delim);
		}
		return 0;
	}
}

bool proceed = true;

void handle_signal(int signum)
{
	if(signum == SIGINT)
	{
		std::cout << "Interrupt signal " << signum << std::endl;
		proceed = false;
	}
}

sdf_fsm::match_params tested_params[] = {
	{
		.proto = 17,
		.source_ip = 3232260200,
		.source_port = 1100,
		.dest_ip = 2886894111,
		.dest_port = 1200
	},
	{
		.proto = 17,
		.source_ip = 3232260360,
		.source_port = 1100,
		.dest_ip = 2886893999,
		.dest_port = 1200
	},
	{
		.proto = 17,
		.source_ip = 3232250096, //miss
		.source_port = 1100,
		.dest_ip = 2886893570,
		.dest_port = 1201

	},
	{
		.proto = 17,
		.source_ip = 3232260999,
		.source_port = 1100,
		.dest_ip = 2886893599,
		.dest_port = 1203
	},
	{
		.proto = 17,
		.source_ip = 3232263333,
		.source_port = 1100,
		.dest_ip = 2886894444,
		.dest_port = 1204
	},
	{
		.proto = 17,
		.source_ip = 3232263896,
		.source_port = 1100,
		.dest_ip = 2886894691, //miss
		.dest_port = 1202
	}
};

struct state;

struct transition {
	sdf_fsm::value_range arg;
	state* to;
	bool operator<(uint32_t value) {
		return arg.range_end < value;
	}
};

struct state {
	void* data = nullptr;
	std::vector<transition> transitions;
	bool is_final = false;
};

void* try_match(sdf_fsm::match_params& match_params, sdf_fsm::on_find_action action, state* fsm_root) {
	state* next = fsm_root;
	bool is_final = next->is_final;
	uint32_t* value = (uint32_t*)(&match_params);
	while(is_final != true) {
		auto search_it = std::lower_bound(next->transitions.begin(), next->transitions.end(), *value);
		next = search_it.base()->to;
		is_final = next->is_final;
		++value;
	}
	if(action != nullptr) {
		action(next->data);
	}
	return next->data;
}

enum cl_option_type {
	e_none = -1,
	e_show_counters,
	e_database_count,
	e_regex_count,
	e_searches,
	e_monitor_memory,
	e_help,
};

char const *help_strings[] = {
	"[true|false] (false) tells application weather to show counters while creating regexes and databases or not",
	"[INT] (1) number of databases to allocate",
	"[INT] (1) number of regular expressions per database to compile",
	"[INT] (6) number of searches to perform per database",
	"[true|false] (true) tells application weather to sleep before performance tests",
	"show this message",
};

char const *long_opts[] = {
	"show_counters",
	"n_db",
	"n_regex",
	"n_searches",
	"monitor_memory",
	"help",
};

char const *short_opts[] = {
	"",
	"d",
	"r",
	"s",
	"m",
	"h",
};

size_t db_count = 1;
size_t regex_count = 1;
size_t n_searches = 6;
bool show_counters = false;
bool monitor_memory = true;

int main(int argc, char* argv[]) {

	//parse command line arguments
	int rc = cl_option::init("range_based_fsm", (sizeof(long_opts) / sizeof(char*)), short_opts, long_opts, help_strings);
	if(rc != 0) {
		std::cerr << "error while cl_options::init()" << std::endl;
		return -1;
	}

	if(argc > 1) {
		for(size_t arg_id = 1; arg_id < argc; ++arg_id) {
			auto has_kvp_v = [=]() -> bool {
				if((arg_id + 1) == argc)
				{
					std::cerr << "incorrect number of arguments" << std::endl;
					cl_option::print_help();
					return false;
				}
				return true;
			};
			switch (cl_option(argv[arg_id]).type)
			{	
			case cl_option_type::e_help:
				cl_option::print_help();
				return 0;
			case cl_option_type::e_regex_count:
				if (not has_kvp_v()) return -1;
				regex_count = atoll(argv[++arg_id]);
				if(regex_count == 0) {
					std::cerr << "error while atoll(" << argv[arg_id] << "). the value must be in range [1," << UINT64_MAX << "]" << std::endl;
					return -1;
				}
				break;
			case cl_option_type::e_database_count:
				if (not has_kvp_v()) return -1;
				db_count = atoll(argv[++arg_id]);
				if(db_count == 0) {
					std::cerr << "error while atoll(" << argv[arg_id] << "). the value must be in range [1," << UINT64_MAX << "]" << std::endl;
					return -1;
				}
				break;
			case cl_option_type::e_show_counters:
				if (not has_kvp_v()) return -1;
				rc = cl_option::stobool(argv[++arg_id], &show_counters);
				if(rc != 0) {
					std::cerr << "error while cl_option::stobool() of " << argv[arg_id]  << std::endl;
					return -1;
				}
				break;
			case cl_option_type::e_searches:
				if (not has_kvp_v()) return -1;
				n_searches = atoll(argv[++arg_id]);
				if(rc != 0) {
					std::cerr << "error while atoll(" << argv[arg_id] << "). the value must be in range [1," << UINT64_MAX << "]" << std::endl;
					return -1;
				}
				break;
			case cl_option_type::e_monitor_memory:
				if (not has_kvp_v()) return -1;
				rc = cl_option::stobool(argv[++arg_id], &monitor_memory);
				if(rc != 0) {
					std::cerr << "error while cl_option::stobool() of " << argv[arg_id]  << std::endl;
					return -1;
				}
				break;
			case cl_option_type::e_none:
			default:
				std::cerr << "unrecognized option type: \"" << argv[arg_id] << "\"" << std::endl;
				cl_option::print_help();
				return -1;
			}
		}
	}

	rc = sdf_fsm::test_sdf_fsm();
	return 0;

	std::string inc_filter = "permit out 17 from 192.168.102.147/20 1100 to 172.18.130.24/22 1200-1205";
	sdf_fsm::filter_params params;
	// std::cout << "parse sdf filter params from " << inc_filter;
	
	rc = sdf_filter::parse_filter_params(inc_filter.data(), &params);
	if(rc != 0) {
		std::cerr << "error while sdf_filter::parse_filter_params()" << std::endl;
		return -1;
	}
	// std::cout << " - OK!" << std::endl;

	params.print_params();
	//create sigint signal handler
	signal(SIGINT, handle_signal);
	
	struct assosiation {
		std::string test = "hello";
		int counter = 0;
	} found_ass;

	std::vector<state> all_roots;
	std::cout << "initialize " << db_count << " databases";
	all_roots.resize(db_count);
	std::cout << " - OK!" << std::endl;
	std::cout << "compile regex databases..." << std::endl;
	for(auto& fsm: all_roots) {
		fsm = state {
			.transitions = std::vector<transition> {
				transition {
					.arg = {
						.range_start = 0,
						.range_end = params.proto.range_start - 1
					},
					.to = new state {
						.data = nullptr,
						.is_final = true
					}
				},
				transition {
					.arg = {
						.range_start = params.proto.range_start,
						.range_end = params.proto.range_end
					},
					.to = new state {
						.transitions = std::vector<transition> {
							transition {
								.arg = {
									.range_start = 0,
									.range_end = params.source_ip.range_start - 1
								},
								.to = new state {
									.data = nullptr,
									.is_final = true
								}
							},
							transition {
								.arg = {
									.range_start = params.source_ip.range_start,
									.range_end = params.source_ip.range_end
								},
								.to = new state {
									.transitions = std::vector<transition> {
										transition {
											.arg = {
												.range_start = 0,
												.range_end = params.source_port.range_start - 1
											},
											.to = new state {
												.data = nullptr,
												.is_final = true
											}
										},
										transition {
											.arg = {
												.range_start = params.source_port.range_start,
												.range_end = params.source_port.range_end
											},
											.to = new state {
												.transitions = std::vector<transition> {
													transition {
														.arg = {
															.range_start = 0,
															.range_end = 2886893567
														},
														.to = new state {
															.data = nullptr,
															.is_final = true
														}
													},
													transition {
														.arg = {
															.range_start = params.dest_ip.range_start,
															.range_end = params.dest_ip.range_end
														},
														.to = new state {
															.transitions = std::vector<transition> {
																transition{
																	.arg = {
																		.range_start = 0,
																		.range_end = params.dest_port.range_start - 1
																	},
																	.to = new state {
																		.data = nullptr,
																		.is_final = true
																	}
																},
																transition{
																	.arg = {
																		.range_start = params.dest_port.range_start,
																		.range_end = params.dest_port.range_end
																	},
																	.to = new state {
																		.data = &found_ass,
																		.is_final = true
																	}
																},
																transition{
																	.arg = {
																		.range_start = params.dest_port.range_end + 1,
																		.range_end = UINT32_MAX
																	},
																	.to = new state {
																		.data = nullptr,
																		.is_final = true
																	}
																}
															}
														}
													},
													transition {
														.arg = {
															.range_start = 2886894592,
															.range_end = UINT32_MAX
														},
														.to = new state {
															.data = nullptr,
															.is_final = true
														}
													}
												}
											}
										},
										transition {
											.arg = {
												.range_start = params.source_port.range_end + 1,
												.range_end = UINT32_MAX
											},
											.to = new state {
												.data = nullptr,
												.is_final = true
											}
										}
									}
								}
							},
							transition {
								.arg = {
									.range_start = params.source_ip.range_end + 1,
									.range_end = UINT32_MAX
								},
								.to = new state {
									.data = nullptr,
									.is_final = true
								}
							}
						}
					}
				},
				transition {
					.arg = {
						.range_start = params.proto.range_end + 1,
						.range_end = UINT32_MAX
					},
					.to = new state {
						.data = nullptr,
						.is_final = true
					}
				}
			}
		};
	}
	std::cout << "... OK!" << std::endl;

	std::cout << "sleep before performance test..." << std::endl;
	if(monitor_memory) {
		while(proceed) {
			sleep(10);
		}
	}
	std::cout << "performance test start" << std::endl;
	auto beg = std::chrono::high_resolution_clock::now();
	for(size_t search_id = 0; (search_id < n_searches) and (proceed); ++search_id) {
		for(size_t db_id = 0; db_id < db_count; ++db_id) {
			void* found = try_match(
				tested_params[search_id%6],
				nullptr,
				&all_roots[db_id]
			);
			if(found != nullptr) {
				++(((assosiation*)found)->counter);
			}
		}
	}
	auto end = std::chrono::high_resolution_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
	std::cout << "duration: " << duration.count() << " milliseconds" << std::endl;
	std::cout << "match speed: " << float(n_searches * db_count) / float(duration.count()) * 1000 << " el/sec" << std::endl;
	// std::cout << "got: " << callee. << " matches" << std::endl;
	std::cout << "matches: " << found_ass.counter << std::endl;

	return 0;

}