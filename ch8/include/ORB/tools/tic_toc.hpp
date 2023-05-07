#ifndef ORB_TOOLS_TIC_TOC_HPP_
#define ORB_TOOLS_TIC_TOC_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace ORB {
    class TicToc {
        public:
            TicToc() {
                tic();
            }
        
            void tic()
            {
                start = std::chrono::steady_clock::now();
            }

            // 返回时间是秒
            double toc()
            {
                end = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
                start = std::chrono::steady_clock::now();
                return elapsed_seconds.count();
            }


        private:
            std::chrono::time_point<std::chrono::steady_clock> start, end;
    };
}

#endif