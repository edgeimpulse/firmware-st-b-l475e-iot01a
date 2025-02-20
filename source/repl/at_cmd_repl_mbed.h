/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EDGE_IMPULSE_AT_COMMANDS_REPL_MBED_H_
#define _EDGE_IMPULSE_AT_COMMANDS_REPL_MBED_H_

#define EI_SIGNAL_TERMINATE_THREAD_REQ      0x1
#define EI_SIGNAL_TERMINATE_THREAD_RES      0x4

/**
 * Mbed OS specific code for handling the REPL and throwing AT
 * commands around...
 */

#include "mbed.h"
#include "mbed_mem_trace.h"
#include "at_cmds.h"
#include "repl.h"

// static void print_memory_info2() {
//     // allocate enough room for every thread's stack statistics
//     int cnt = osThreadGetCount();
//     mbed_stats_stack_t *stats = (mbed_stats_stack_t*) malloc(cnt * sizeof(mbed_stats_stack_t));

//     cnt = mbed_stats_stack_get_each(stats, cnt);
//     for (int i = 0; i < cnt; i++) {
//         printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
//     }
//     free(stats);

//     // Grab the heap statistics
//     mbed_stats_heap_t heap_stats;
//     mbed_stats_heap_get(&heap_stats);
//     printf("Heap size: %lu / %lu bytes (max: %lu)\r\n", heap_stats.current_size, heap_stats.reserved_size, heap_stats.max_size);
// }


typedef struct {
    char cmd[256];
} ei_at_mail_t;

class AtCmdRepl {
public:
    AtCmdRepl(EventQueue *queue = NULL, uint32_t cmd_thread_stack_size = OS_STACK_SIZE, unsigned char *cmd_thread_stack_mem = NULL)
        : _cmd_thread_stack_size(cmd_thread_stack_size),
          _cmd_thread_stack_mem(cmd_thread_stack_mem),
          _queue(queue),
          _repl(queue),
          _mail_handled(0),
          _terminate_thread(false)
    {
    }

    /**
     * Start the REPL
     * (note that you'll need to run the event queue as well!)
     */
    void start_repl() {
        // we declare this on the heap as we need to be able to dispose and restart the thread
        // and Mbed cannot start a stopped thread
        _cmd_thread = new Thread(osPriorityNormal, _cmd_thread_stack_size, _cmd_thread_stack_mem, "at-cmd-thread");
        _cmd_thread->start(callback(this, &AtCmdRepl::cmd_thread_main));

        _repl.start(callback(this, &AtCmdRepl::exec_command));
    }

    /**
     * When executing a command we'll detach the UART interrupts from the REPL
     * and handle them ourselves, to allow breaking off a command...
     */
    bool exec_command(const char *cmd) {
        if (strlen(cmd) > 255) {
            printf("Command is too long (max. length 255)\n");
            return true;
        }

        _terminate_thread = false;

        _repl.stop();
        _repl.getSerial()->attach(callback(this, &AtCmdRepl::break_inference_loop_irq));

        // execution is done on a different thread (so we can cancel it)
        // so send msg to other thread and wait for signal to come back
        ei_at_mail_t *mail = _mail_box.alloc();
        if (!mail) {
            printf("Failed to allocate item in mail box!\n");
            _repl.start(callback(this, &AtCmdRepl::exec_command));
            return false;
        }
        memcpy(mail->cmd, cmd, strlen(cmd) + 1);
        _mail_box.put(mail);

        // wait for the semaphore to fire
        _mail_handled.acquire();

        // event was cancelled
        if (_terminate_thread) {
            osPriority_t priority = _cmd_thread->get_priority();

            // signal the other thread, and wait until it's done
            _cmd_thread->signal_set(EI_SIGNAL_TERMINATE_THREAD_REQ);

            for (size_t ix = 0; ix < 10; ix++) {
                ThisThread::sleep_for(200);
                if (_cmd_thread->get_state() != Thread::State::Ready) {
                    break;
                }
            }

            _cmd_thread->terminate();

            delete _cmd_thread;

            // have to restart the thread... have to make new one, because Mbed OS does not allow restarting terminated threads
            _cmd_thread = new Thread(priority, _cmd_thread_stack_size, _cmd_thread_stack_mem, "at-cmd-thread");
            _cmd_thread->start(callback(this, &AtCmdRepl::cmd_thread_main));
        }

        _mail_box.free(mail);

        // and command is handled, restart REPL
        _repl.start(callback(this, &AtCmdRepl::exec_command));

        _terminate_thread = false;

        return false; // don't reprint state (already done when starting the repl again)
    }

    void break_inference_loop_irq() {
        if (_repl.getSerial()->readable() && _repl.getSerial()->getc() == 'b') {
            _terminate_thread = true;

            // release the semaphore so the main thread knows the event is done
            _mail_handled.release();
        }
    }

    void reprintReplState() {
        _repl.reprintReplState();
    }

private:
    /**
     * Main thread for commands, pretty straight forward
     * it will wait for an item in the mailbox, execute it, then use the semaphore to signal back that we're done
     */
    void cmd_thread_main() {
        while (1) {
            osEvent evt = _mail_box.get();
            if (evt.status == osEventMail) {
                ei_at_mail_t *mail = (ei_at_mail_t*)evt.value.p;

                ei_at_cmd_handle(mail->cmd);

                // signal to other thread
                if (!_terminate_thread) {
                    // so this shouldn't be an issue I'd say but release'ing this again
                    // (whilst it was also released in break_inference_loop_irq)
                    // seems to break the state (and we can no longer aquire this semaphore later)
                    _mail_handled.release();
                }
            }
        }
    }

    uint32_t _cmd_thread_stack_size;
    unsigned char *_cmd_thread_stack_mem;
    EventQueue *_queue;
    Repl _repl;
    Mail<ei_at_mail_t, 1> _mail_box;
    Semaphore _mail_handled;
    Thread *_cmd_thread;
    bool _terminate_thread;
};

#endif // _EDGE_IMPULSE_AT_COMMANDS_REPL_MBED_H_
