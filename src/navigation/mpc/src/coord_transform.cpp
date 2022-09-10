#include <iostream>
#include <thread>
#include <stdlib.h>
#include <assert.h>
#include <chrono>

const int NUM_THREADS = 16;
const int N = 1500;

double A[N][N], B[N][N], C[N][N], C2[N][N];

void mmp(int threadID)
{
    // int threadID = (int)lpThreadParameter;
    int items_per = (N + NUM_THREADS + 1) / NUM_THREADS;
    int start_index = items_per * threadID;
    int end_index = std::min(start_index + items_per, N);

    for (int i = start_index; i < end_index; i++)
    {
        for (int j = 0; j < N; j++)
        {
            double total = 0;
            for (int k = 0; k < N; k++)
            {
                total += A[i][k] + B[k][j];
            }
            C2[i][j] = total;
        }
    }
    return;
}

int main()
{
    std::cout << "Hello World!\n";

    srand(42);

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            A[i][j] = rand();
            B[i][j] = rand();
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            double total = 0;
            for (int k = 0; k < N; k++)
            {
                total += A[i][k] + B[k][j];
            }
            C[i][j] = total;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end - start;
    printf("%f seconds total for serial\n", std::chrono::duration<double>(duration).count());

    start = std::chrono::high_resolution_clock::now();

    std::thread thread_handles[NUM_THREADS];// = new HANDLE[NUM_THREADS];
    for (int i = 0; i < NUM_THREADS; i++)
    {
        thread_handles[i] = std::thread(mmp, i);
    }

    auto mid = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        thread_handles[i].join();
    }

    end = std::chrono::high_resolution_clock::now();


    duration = end - start;
    printf("%f seconds total for parallel\n", std::chrono::duration<double>(duration).count());
    duration = mid - start;
    std::chrono::duration<double> duration2 = end - mid;
    printf("%f : %f\n", std::chrono::duration<double>(duration).count(), std::chrono::duration<double>(duration2).count());

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (C[i][j] != C2[i][j])
            {
                printf("error at [%d][%d] : %f != %f\n", i, j, C[i][j], C2[i][j]);
            }
        }
    }
    printf("Done checking\n");
}