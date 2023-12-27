#pragma once


template <typename T>
void wait_all(T&& futures) {
	for (auto& f : futures) {
		f.get();
	}
}


template <typename Int, typename F>
void parallel_for_async(Int start, Int end, F body, 
    std::vector<std::future<void>>& futures) {
    Int local_num_threads = (Int)m_num_threads;

    Int range = end - start;
    Int chunk = (range / local_num_threads) + 1;

    for (Int i = 0; i < local_num_threads; ++i) {
        futures.emplace_back(enqueue_task([i, chunk, start, end, body] {
            Int inner_start = start + i * chunk;
            Int inner_end = std::min(end, start + (i + 1) * chunk);
            for (Int j = inner_start; j < inner_end; ++j) {
                body(j);
            }
        }));
    }
}