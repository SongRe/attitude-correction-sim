#include "control_proxy.h"
#include <pthread.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
cntrl_proxy cntrl_proxy_init(const att_cntrl_data *comm, const att_cntrl_data *curr)
{
    return (cntrl_proxy){
        .comm = comm,
        .curr = curr};
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
void cntrl_proxy_init_protect(cntrl_proxy *proxy, const pthread_mutex_t *comm_protect, const pthread_mutex_t *curr_protect)
{
    proxy->comm_protect = comm_protect;
    proxy->curr_protect = curr_protect;
}
#pragma GCC diagnostic pop

void cntrl_proxy_pull_comm(const cntrl_proxy *proxy, att_cntrl_data *comm_repl)
{
    pthread_mutex_lock(proxy->comm_protect);
    comm_repl->attit = proxy->comm->attit;
    comm_repl->ang_vel = proxy->comm->ang_vel;
    pthread_mutex_unlock(proxy->comm_protect);
}

void cntrl_proxy_pull_curr(const cntrl_proxy *proxy, att_cntrl_data *curr_repl)
{
    pthread_mutex_lock(proxy->curr_protect);
    curr_repl->attit = proxy->curr->attit;
    curr_repl->ang_vel = proxy->curr->ang_vel;
    pthread_mutex_unlock(proxy->curr_protect);
}
