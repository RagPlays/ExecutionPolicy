#include <iostream>
#include <cassert>
#include <vector>
#include <chrono>
#include <random>
#include <ranges>

#include <thread>
#include <future>
#include <mutex>
#include <execution>

// CountPointVariables //
static int s_In{};
static int s_Total{};
static std::mutex s_Mutex{};
static std::atomic<int> s_InAtomic{};
static std::atomic<int> s_TotalAtomic{};

// Other Variables //
static const unsigned int s_NrThreads{ std::thread::hardware_concurrency() };
static constexpr unsigned int s_NrSamples{ 1'000'000 };
static constexpr unsigned int s_NrMeasurements{ 10 };
static const unsigned int s_PointsPerThread{ s_NrSamples / s_NrThreads };

// Time
static std::chrono::high_resolution_clock::time_point s_MeasureStart{};
static std::chrono::high_resolution_clock::time_point s_MeasureEnd{};
static std::chrono::duration<double, std::milli> s_MeasureDiff{};

struct Vector2f
{
    float x;
    float y;
};

struct MeasureData
{
    unsigned int time;
    float result;
};

class MeasureHelper final
{
public:

    MeasureHelper(unsigned int capacity)
    {
        m_Data.reserve(capacity);
    }
    ~MeasureHelper() = default;

    MeasureData GetAverageMeasure()
    {
        // Check
        if (m_Data.size() < 3)
        {
            assert(false);
            return MeasureData{};
        }

        // Sort the data
        std::ranges::sort(m_Data,
            [](const MeasureData& dataA, const MeasureData& dataB)
            {
                return dataA.time < dataB.time;
            }
        );

        // Remove last and first element (min and max)
        m_Data.erase(m_Data.begin());
        m_Data.erase(m_Data.end() - 1);

        // Get Average
        unsigned int sumTime{};
        float sumResult{};
        unsigned int count{};
        for (const auto& data : m_Data)
        {
            sumTime += data.time;
            sumResult += data.result;
            ++count;
        }

        if (m_Data.empty())
        {
            int i{};
            ++i;
        }

        const unsigned int averageTime{ sumTime / count };
        const float averageResult{ sumResult / count };

        return MeasureData{ averageTime, averageResult };
    }

    void AddData(const MeasureData& data)
    {
        m_Data.emplace_back(data);
    }

private:
    std::vector<MeasureData> m_Data;
};

Vector2f GetRandomPoint()
{
    thread_local std::random_device rd{};
    thread_local std::default_random_engine eng{ rd() };
    thread_local std::uniform_real_distribution<float> distr{ -1, 1 };
    return { distr(eng), distr(eng) };
}

void CountPoints(int number)
{
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++s_In;
        }
    }
    s_Total += number;
}

void CountPointsMutex(int number)
{
    int localIn{};
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++localIn;
        }
    }

    std::lock_guard<std::mutex> lock(s_Mutex);
    s_In += localIn;
    s_Total += number;
}

void CountPointsAtomic(int number)
{
    int localIn{};
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++localIn;
        }
    }
    s_InAtomic += localIn;
    s_TotalAtomic += number;
}

void CountPointsPromise(int number, std::promise<int>* promise)
{
    int localIn{};
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++localIn;
        }
    }
    promise->set_value(localIn);
}

int CountPointsAsync(int number)
{
    int localIn{};
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++localIn;
        }
    }
    return localIn;
}

void CountPointsSTL(int number)
{
    int localIn{};
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++localIn;
        }
    }

    std::lock_guard<std::mutex> lock(s_Mutex);
    s_In += localIn;
    s_Total += number;
}

void PrintTime(const std::string& version, int milisec, float result)
{
    std::cout << version << " version: " << milisec << " ms\n";
    std::cout << "result: " << result << "\n\n";
}

void RunOriginalVersion()
{
    MeasureHelper helper{ s_NrMeasurements };

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        s_In = 0;
        s_Total = 0;

        s_MeasureStart = std::chrono::high_resolution_clock::now();

        CountPoints(s_NrSamples);

        s_MeasureEnd = std::chrono::high_resolution_clock::now();
        s_MeasureDiff = s_MeasureEnd - s_MeasureStart;
        helper.AddData(MeasureData{static_cast<unsigned int>(s_MeasureDiff.count()), 4.f * s_In / s_Total });
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Original", measuredData.time, measuredData.result);
}

void RunMutexVersion()
{
    MeasureHelper helper{ s_NrMeasurements };

    std::vector<std::jthread> threads{};

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        s_In = 0;
        s_Total = 0;
        threads.clear();
        threads.resize(s_NrThreads);

        s_MeasureStart = std::chrono::high_resolution_clock::now();

        for (auto& thread : threads)
        {
            thread = std::jthread(CountPointsMutex, s_PointsPerThread);
        }
        for (auto& thread : threads)
        {
            thread.join();
        }

        s_MeasureEnd = std::chrono::high_resolution_clock::now();
        s_MeasureDiff = s_MeasureEnd - s_MeasureStart;
        helper.AddData(MeasureData{ static_cast<unsigned int>(s_MeasureDiff.count()), 4.f * s_In / s_Total });
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Mutex", measuredData.time, measuredData.result);
}

void RunAtomicVersion()
{
    MeasureHelper helper{ s_NrMeasurements };

    std::vector<std::jthread> threads{};

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        s_InAtomic = 0;
        s_TotalAtomic = 0;
        threads.clear();
        threads.resize(s_NrThreads);

        s_MeasureStart = std::chrono::high_resolution_clock::now();

        for (auto& thread : threads)
        {
            thread = std::jthread(CountPointsAtomic, s_PointsPerThread);
        }
        for (auto& thread : threads)
        {
            thread.join();
        }

        s_MeasureEnd = std::chrono::high_resolution_clock::now();
        s_MeasureDiff = s_MeasureEnd - s_MeasureStart;
        helper.AddData(MeasureData{ static_cast<unsigned int>(s_MeasureDiff.count()), 4.f * s_InAtomic / s_TotalAtomic });
    }
    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Atomic", measuredData.time, measuredData.result);
}

void RunPromiseVersion()
{
    MeasureHelper helper{ s_NrMeasurements };

    std::vector<std::jthread> threads{};
    std::vector<std::promise<int>> inPromises{};

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        s_In = 0;
        s_Total = 0;
        threads.clear();
        threads.resize(s_NrThreads);
        inPromises.clear();
        inPromises.resize(s_NrThreads);

        s_MeasureStart = std::chrono::high_resolution_clock::now();

        for (unsigned int threadIdx{}; threadIdx < s_NrThreads; ++threadIdx)
        {
            threads[threadIdx] = std::jthread(CountPointsPromise, s_PointsPerThread, &inPromises[threadIdx]);
        }
        for (unsigned int threadIdx{}; threadIdx < s_NrThreads; ++threadIdx)
        {
            threads[threadIdx].join();

            s_In += inPromises[threadIdx].get_future().get();
            s_Total += s_PointsPerThread;
        }
        
        s_MeasureEnd = std::chrono::high_resolution_clock::now();
        s_MeasureDiff = s_MeasureEnd - s_MeasureStart;
        helper.AddData(MeasureData{ static_cast<unsigned int>(s_MeasureDiff.count()), 4.f * s_In / s_Total });
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Promise", measuredData.time, measuredData.result);
}

void RunAsyncVersion()
{
    MeasureHelper helper{ s_NrMeasurements };

    std::vector<std::future<int>> futures{};

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        s_In = 0;
        s_Total = 0;
        futures.clear();
        futures.resize(s_NrThreads);

        s_MeasureStart = std::chrono::high_resolution_clock::now();

        for (unsigned int threadIdx{}; threadIdx < s_NrThreads; ++threadIdx)
        {
            futures[threadIdx] = std::async(std::launch::async, CountPointsAsync, s_PointsPerThread);
        }
        for (unsigned int threadIdx{}; threadIdx < s_NrThreads; ++threadIdx)
        {
            s_In += futures[threadIdx].get();
            s_Total += s_PointsPerThread;
        }

        s_MeasureEnd = std::chrono::high_resolution_clock::now();
        s_MeasureDiff = s_MeasureEnd - s_MeasureStart;
        helper.AddData(MeasureData{ static_cast<unsigned int>(s_MeasureDiff.count()), 4.f * s_In / s_Total });
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Async", measuredData.time, measuredData.result);
}

template<typename ExecPolicy>
void RunSTLVersion(ExecPolicy policy, const std::string& version)
{
    MeasureHelper helper{ s_NrMeasurements };

    std::vector<int> numbers( static_cast<int>(s_NrThreads), static_cast<int>(s_PointsPerThread) );

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        s_In = 0;
        s_Total = 0;

        s_MeasureStart = std::chrono::high_resolution_clock::now();

        std::for_each(policy, numbers.begin(), numbers.end(),
            [](int number)
            {
                CountPointsSTL(number);
            }
        );

        s_MeasureEnd = std::chrono::high_resolution_clock::now();
        s_MeasureDiff = s_MeasureEnd - s_MeasureStart;
        helper.AddData(MeasureData{ static_cast<unsigned int>(s_MeasureDiff.count()), 4.f * s_In / s_Total });
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime(version, measuredData.time, measuredData.result);
}

int main()
{
    // Print Info
    std::cout << "# threads: " << s_NrThreads << "\n";
    std::cout << "# samples: " << s_NrSamples << "\n";
    std::cout << "# measurements: " << s_NrMeasurements << "\n";
    std::cout << "\n";

    const std::chrono::high_resolution_clock::time_point start{ std::chrono::high_resolution_clock::now() };

    // Original Version //
    RunOriginalVersion();

    // Mutex Version //
    RunMutexVersion();

    // Atomic Version //
    RunAtomicVersion();

    // Promise - Future Version //
    RunPromiseVersion();

    // Async Version //
    RunAsyncVersion();

    // STL seq Version //
    RunSTLVersion(std::execution::seq, "STL seq");

    // STL par Version //
    RunSTLVersion(std::execution::par, "STL par");

    // STL par_unseq Version //
    RunSTLVersion(std::execution::par_unseq, "STL par_unseq");

    // STL unseq Version //
    RunSTLVersion(std::execution::unseq, "STL unseq");

    const std::chrono::high_resolution_clock::time_point end{ std::chrono::high_resolution_clock::now() };
    const std::chrono::duration<double> diff{ end - start };

    std::cout << "Total operational time: " << diff.count() << " sec\n";

    return 0;
}