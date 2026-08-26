#include "engine.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <numeric>
#include <iomanip>

using namespace route_sim;

int main(int argc, char* argv[]) {
    std::string dataset_path;
    std::string mode = "single";
    int workers = 8;
    int warmup = 0;
    int runs = 1;
    std::string output_path;
    std::string algo_override = "ALL";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dataset" && i + 1 < argc) dataset_path = argv[++i];
        else if (arg == "--mode" && i + 1 < argc) mode = argv[++i];
        else if (arg == "--workers" && i + 1 < argc) workers = std::stoi(argv[++i]);
        else if (arg == "--warmup" && i + 1 < argc) warmup = std::stoi(argv[++i]);
        else if (arg == "--runs" && i + 1 < argc) runs = std::stoi(argv[++i]);
        else if (arg == "--output" && i + 1 < argc) output_path = argv[++i];
        else if (arg == "--algorithm" && i + 1 < argc) algo_override = argv[++i];
    }

    if (dataset_path.empty()) {
        std::cerr << "Usage: " << argv[0] << " --dataset <path> [--mode single|multi] [--workers N] [--warmup N] [--runs N] [--output <path>] [--algorithm DIJKSTRA|ASTAR|ALL]\n";
        return 1;
    }

    auto t_load_start = std::chrono::high_resolution_clock::now();
    auto [graph, queries] = load_dataset(dataset_path);
    auto t_load_end = std::chrono::high_resolution_clock::now();
    double load_time_ms = std::chrono::duration<double, std::milli>(t_load_end - t_load_start).count();

    if (algo_override != "ALL") {
        for (auto& q : queries) {
            q.algorithm = algo_override;
        }
    }

    // Warmup
    for (int w = 0; w < warmup; ++w) {
        if (mode == "single") {
            auto _ = run_sequential(graph, queries);
        } else {
            auto _ = run_parallel(graph, queries, workers);
        }
    }

    // Benchmark runs
    std::vector<double> run_times_ms;
    std::vector<RouteResult> last_results;

    for (int r = 0; r < runs; ++r) {
        auto t_start = std::chrono::high_resolution_clock::now();
        if (mode == "single") {
            last_results = run_sequential(graph, queries);
        } else {
            last_results = run_parallel(graph, queries, workers);
        }
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        run_times_ms.push_back(elapsed);
    }

    double sum_time = std::accumulate(run_times_ms.begin(), run_times_ms.end(), 0.0);
    double avg_time_ms = sum_time / run_times_ms.size();
    double min_time_ms = *std::min_element(run_times_ms.begin(), run_times_ms.end());
    double max_time_ms = *std::max_element(run_times_ms.begin(), run_times_ms.end());
    double throughput_qps = avg_time_ms > 0.0 ? (queries.size() * 1000.0) / avg_time_ms : 0.0;

    // Output JSON serialization if requested
    if (!output_path.empty()) {
        std::ofstream out(output_path);
        out << "{\n";
        out << "  \"language\": \"cpp\",\n";
        out << "  \"dataset\": \"" << dataset_path.substr(dataset_path.find_last_of("/\\") + 1) << "\",\n";
        out << "  \"mode\": \"" << mode << "\",\n";
        out << "  \"workers\": " << (mode == "multi" ? workers : 1) << ",\n";
        out << "  \"dataset_load_time_ms\": " << load_time_ms << ",\n";
        out << "  \"num_queries\": " << queries.size() << ",\n";
        out << "  \"runs\": " << runs << ",\n";
        out << "  \"avg_time_ms\": " << avg_time_ms << ",\n";
        out << "  \"min_time_ms\": " << min_time_ms << ",\n";
        out << "  \"max_time_ms\": " << max_time_ms << ",\n";
        out << "  \"query_throughput_qps\": " << throughput_qps << ",\n";
        out << "  \"results\": [\n";

        for (size_t i = 0; i < last_results.size(); ++i) {
            const auto& res = last_results[i];
            out << "    {\n";
            out << "      \"source\": " << res.source << ",\n";
            out << "      \"destination\": " << res.destination << ",\n";
            out << "      \"optimization\": \"" << res.optimization << "\",\n";
            out << "      \"algorithm\": \"" << res.algorithm << "\",\n";
            out << "      \"route\": [";
            for (size_t k = 0; k < res.route.size(); ++k) {
                out << res.route[k] << (k + 1 < res.route.size() ? ", " : "");
            }
            out << "],\n";
            out << "      \"distance_km\": " << std::fixed << std::setprecision(4) << res.distance_km << ",\n";
            out << "      \"travel_time_min\": " << res.travel_time_min << ",\n";
            out << "      \"fuel_liters\": " << res.fuel_liters << ",\n";
            out << "      \"fuel_cost\": " << res.fuel_cost << ",\n";
            out << "      \"toll_cost\": " << res.toll_cost << ",\n";
            out << "      \"total_cost\": " << res.total_cost << ",\n";
            out << "      \"nodes_explored\": " << res.nodes_explored << ",\n";
            out << "      \"execution_time_us\": " << res.execution_time_us << "\n";
            out << "    }" << (i + 1 < last_results.size() ? "," : "") << "\n";
        }
        out << "  ]\n}\n";
    }

    std::cout << "{\"language\": \"cpp\", \"mode\": \"" << mode << "\", \"num_queries\": " << queries.size()
              << ", \"avg_time_ms\": " << avg_time_ms << ", \"query_throughput_qps\": " << throughput_qps << "}\n";

    return 0;
}
