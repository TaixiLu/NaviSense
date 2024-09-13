
#include "Fitful_IO.h"

Fitful_IO_base::Fitful_IO_base()
{

    heartbeat_Semaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(Fitful_IO_base::heartbeat, "GUI throttle heartbeat",
                            2000 + configMINIMAL_STACK_SIZE, this, 10,
                            &heartbeat_task_handle, 0);
    set_state(false);
}
void Fitful_IO_base::heartbeat(void *param)
{
    Fitful_IO_base *p_fithful_io = (Fitful_IO_base *)param;
    while (true)
    {
        if (xSemaphoreTake(p_fithful_io->heartbeat_Semaphore, portMAX_DELAY) == pdTRUE)
        {
            while (!p_fithful_io->state_queue.empty())
            {
                stuct_io_seq curr_seq = p_fithful_io->state_queue.front();
                p_fithful_io->set_state(true);
                vTaskDelay(pdMS_TO_TICKS(curr_seq.on_delay));
                if (!p_fithful_io->state_queue.empty())
                {
                    p_fithful_io->set_state(false);
                    vTaskDelay(pdMS_TO_TICKS(curr_seq.off_delay));
                    if (!p_fithful_io->state_queue.empty())
                        p_fithful_io->state_queue.pop();
                }
            }
        }
    }
}

void Fitful_IO_base::begin(uint32_t on_delay, uint32_t off_delay, uint8_t count_down)
{
    if (!state_queue.empty())
    {
        std::queue<stuct_io_seq>().swap(state_queue);
        xTaskAbortDelay(heartbeat_task_handle);
        while (eRunning == eTaskGetState(heartbeat_task_handle))
            vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (count_down != 0)
    {
        add(on_delay, off_delay, count_down);
        xSemaphoreGive(heartbeat_Semaphore);
    }
}
void Fitful_IO_base::add(uint32_t on_delay, uint32_t off_delay, uint8_t count_down)
{
    stuct_io_seq tmp_seq;
    tmp_seq.on_delay = on_delay;
    tmp_seq.off_delay = off_delay;
    for (int i = 0; i < count_down; i++)
        state_queue.push(tmp_seq);
}

bool Fitful_IO_base::is_busy()
{
    return !state_queue.empty();
}
