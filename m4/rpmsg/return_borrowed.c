#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_mu.h"

#define SIZE 256
#define RPMSG_ADDR 0xb8000000
#define VRING_TOTAL 0x8000
#define RPMSG_NAME_SIZE 32

struct descriptor {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
};

struct vring_avail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[SIZE];
};

struct vring_used_elem {
    uint16_t id;
    uint32_t len;
};

struct vring_used {
    uint16_t flags;
    uint16_t idx;
    struct vring_used_elem used[SIZE];
};

struct rpmsg_message {
    uint32_t src;
    uint32_t dst;
    uint32_t res;
    uint16_t len;
    uint16_t flags;
};

struct rpmsg_channel_info {
    char name[RPMSG_NAME_SIZE];
    uint32_t src;
    uint32_t dst;
};

struct vring {
    // pointers to the shared structures
    struct descriptor *desc;
    struct vring_avail *avail;
    struct vring_used *used;

    // local indexes that need to persist across m4 restarts
    uint16_t head_avail;

    // copied index from irq to our local memory 
    volatile uint16_t idx_avail;
    volatile uint16_t idx_used;
};

struct rpmsg_shared {
    // initialize iif magic doesnt match
    int magic;
    struct vring tx;
    struct vring rx;

    // 1 if linux buffer is borrowed and needs to be returned back
    uint8_t borrowed[SIZE];
};

struct rpmsg_shared *rpmsg = RPMSG_ADDR + VRING_TOTAL * 2;

void vring_init(struct vring* vring, int n, int recreate) {
    vring->desc = (struct descriptor*) (RPMSG_ADDR + VRING_TOTAL * n);
    vring->avail = (struct vring_avail*) (RPMSG_ADDR + VRING_TOTAL * n + sizeof(struct descriptor) * SIZE);
    vring->used = (struct vring_used*) (RPMSG_ADDR + VRING_TOTAL * n + 4096 * 2);

    if(recreate == 1) {
        vring->head_avail = 0;

        vring->idx_avail = 0;
        vring->idx_used = 0;
    } else {
        printf("reuse: %d\r\n", vring->head_avail);
    }
}

int rpmsg_init(struct rpmsg_shared *rpmsg) {
    int magic = 0xABCDE4;
    int recreate = rpmsg->magic != magic;
    if(recreate) {
        printf("initializing rpmsg for first time\r\n");
        memset(rpmsg->borrowed, 0, SIZE);
    }
    vring_init(&rpmsg->tx, 0, recreate);
    vring_init(&rpmsg->rx, 1, recreate);
    rpmsg->magic = magic;

    return recreate;
}

void MU_M4_IRQHandler() {
    int data = MU_ReceiveMsgNonBlocking(MUB, 1);
    if(data == 0) {
        rpmsg->tx.idx_avail = rpmsg->tx.avail->idx;
    } else if(data == 65536) {
        rpmsg->rx.idx_avail = rpmsg->rx.avail->idx;
    } else {
        printf("unhandled irq\r\n");
    }
}

uint16_t get_tx_buffer() {
    while(rpmsg->tx.head_avail == rpmsg->tx.idx_avail);
    uint16_t val = rpmsg->tx.avail->ring[rpmsg->tx.head_avail % SIZE];
    rpmsg->tx.head_avail++;

    return val;
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("app loaded\r\n");

    int recreate = rpmsg_init(rpmsg);

    MU_Init(MUB);
    MU_EnableInterrupts(MUB, (1U << 27U) >> 1);
    NVIC_EnableIRQ(MU_M4_IRQn);

    // announce channel only once
    if(recreate || 1) {
        uint16_t id = get_tx_buffer();

        struct rpmsg_message *msg = rpmsg->tx.desc[id].addr;
        msg->src = 0x1e;
        msg->dst = 0x35;
        msg->res = 0;
        msg->len = sizeof(struct rpmsg_channel_info);
        msg->flags = 0x28;

        struct rpmsg_channel_info *info = rpmsg->tx.desc[id].addr + sizeof(struct rpmsg_message);
        //strcpy(info->name, "m4-channel");
        strcpy(info->name, "rpmsg-virtual-tty-channel-1");
        info->src = 0x1e;
        info->dst = 0x00;
       
        rpmsg->tx.used->used[rpmsg->tx.used->idx % SIZE].id = id;
        rpmsg->tx.used->idx++;
        MU_SendMsg(MUB, 1, 0);
    }

    // return not-yet-returned (borrowed) buffers from previous run
    for(int i = 0; i < SIZE; i++) {
        if(rpmsg->borrowed[i]) {
            printf("returning borrowed descriptor %d\r\n", i);
            rpmsg->rx.used->used[rpmsg->rx.used->idx % SIZE].id = i;
            rpmsg->rx.used->idx++;
            rpmsg->borrowed[i] = 0;
            MU_SendMsg(MUB, 1, 65536);
        }
    }

    int i = 0;
    for(;;) {
        // wait for the buffer
        while(rpmsg->rx.head_avail == rpmsg->rx.idx_avail);
        printf("%d %d\r\n", rpmsg->rx.head_avail, rpmsg->rx.idx_avail);
        uint16_t desc_id = rpmsg->rx.avail->ring[rpmsg->rx.head_avail % SIZE] % SIZE;
        
        // mark buffer as borrowed
        rpmsg->borrowed[desc_id] = 1;
        rpmsg->rx.head_avail++;

        char *p = rpmsg->rx.desc[desc_id].addr;
        int len = ((struct rpmsg_message*) rpmsg->rx.desc[desc_id].addr)->len;
        char *payload = p + sizeof(struct rpmsg_message);
        payload[len] = 0;
        printf("%d '%s'\r\n", desc_id, payload);

        i++;

        // return used buffer back to the linux
        /* for now they are not returned 
        rpmsg->rx.used->used[rpmsg->rx.used->idx % SIZE].id = desc_id;
        rpmsg->rx.used->idx++;
        rpmsg->borrowed[desc_id] = 0;
        MU_SendMsg(MUB, 1, 65536);*/
    }

    for(;;);
}
