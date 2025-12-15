#include <global.h>
#include <register.h>

extern void fv_nand_erase(word iw_head_blk);
extern void fv_nand_read(void);
extern void fv_nand_n4k(byte ib_n4rd_idx, byte ib_cmd_hit);
#ifndef CPUA_PROC
extern void fv_nand_htprg(byte ib_cmd_end, dwrd id_cmd_bptr, dwrd id_cmd_padr);
#endif
extern void fv_ftl_oth(void);
#define GROUPSIZE 256
#define BUCKETNUM 16
#define BUFFERSIZE 512
#define THRESHOLD 1000000
#define GROUPNUM 4547664
extern dwrd gd_ht_token;
extern dwrd gd_gc_token;
extern dwrd gd_ht_weight;
extern writebuf* writebuffer;
extern dwrd buffernum;

byte appear[GROUPSIZE];
dwrd writeinit;//多于1000000时，清除学习段
level *gd_l2p_section; //leaFTL:section
#ifdef FTL_DBG
dwrd *gd_p2l_tbl;
dwrd *gd_vmap_tbl;
byte *gb_crl_tbl;
#endif

dwrd gd_vcnt_tbl[RBLK_QNTY];
dwrd gd_ecnt_tbl[RBLK_QNTY];
//dwrd gd_rcnt_tbl[RBLK_QNTY];
//qwrd gq_sn_tbl[RBLK_QNTY];
DBL_TBL gs_dbl_tbl[RBLK_QNTY];
byte gb_blk_type[RBLK_QNTY];

word gw_fblk_str;
word gw_fblk_end;
word gw_fblk_num;
int inhash;
#ifdef GC_SIM
byte gb_htp2l_num = 0;
#endif

PHY_ADR gs_head_padr[HEAD_QNTY];

word gw_htp2l_bfp = 8;
word gw_htp2l_ofs = 0;
dwrd gd_htp2l_ladr[4];
byte new_end;
byte old_end;
byte common_end;
byte common_start ; 
byte gb_htwl_sel = 0;

#ifdef FTL_DBG
dwrd gd_htp2l_ptr;
#endif

#ifdef N4KA_EN
byte gb_n4rd_wptr = 254;
byte gb_n4rd_rptr = 0;
byte gb_n4rd_idxq[255];
byte gb_n4rd_strm[255];
dwrd gd_n4rd_ladr[255];
dwrd gd_n4rd_padr[255];
#endif



#ifdef N4KA_EN
byte fb_ploc_chk(dwrd id_cmd_padr)
{
    byte lb_cmd_cech;
    word lw_cmd_hblk, lw_cmd_page;

    rb_head_idx = GC_HLOC;
    ARM_NOP(); //wait 3 cycle for HW handle

    //check blk
    lw_cmd_hblk = (word)(id_cmd_padr >> (PAGE_SHIFT + BKCH_SHIFT + FRAG_SHIFT));
    if(lw_cmd_hblk != rw_hblk_loc)
    {
        return 0;
    }

    //check page
    lw_cmd_page = (word)(id_cmd_padr >> (BKCH_SHIFT + FRAG_SHIFT)) & PAGE_MASK;
    if(lw_cmd_page/3 != rw_page_loc/3) //if micron b47r/b58r, need change this condition
    {
        return 0;
    }

    //check cech
    lb_cmd_cech = (byte)(id_cmd_padr >> FRAG_SHIFT) & BKCH_MASK;
    if(lb_cmd_cech != rb_cech_loc)
    {
        return 0;
    }

    return 1;
}
#endif


void Insert(dwrd groupidx, section sec, byte level) {
    // 获取目标层级结构
    
    levelsec* lvlsec;
    byte i;
   
    lvlsec = &gd_l2p_section[groupidx].levelsecgroup[level];
    
    // 遍历当前层级的所有section，检查冲突
    for (i = 0; i < lvlsec->number; i++) {
        section* old_sec ;
        old_sec= &lvlsec->secgroup[i];
        
        // 计算新旧section的范围
        
        old_end= old_sec->start + old_sec->length;
        
        new_end= sec.start + sec.length;
        
        // 检查是否有范围重叠
        if (!(new_end <= old_sec->start || sec.start >= old_end)) {
            // 存在冲突
            
            // 检查是否满足合并条件：step相同且有共同元素
            if (old_sec->step == sec.step) {
                // 计算共同元素范围
                
                common_start=(sec.start > old_sec->start) ? sec.start : old_sec->start;
                 
                common_end= (new_end < old_end) ? new_end : old_end;
                
                // 检查是否有共同元素
                if (common_start < common_end) {
                    // 可以合并，更新旧的section
                    // 新的起始位置取更小的
                    if (sec.start < old_sec->start) {
                        // 需要扩展旧的section到前面
                        dwrd lba_diff; 
                        lba_diff= old_sec->start - sec.start;
                        old_sec->b = sec.b + (lba_diff * sec.step);
                        old_sec->start = sec.start;
                    }
                    
                    // 新的结束位置取更大的
                    if (new_end > old_end) {
                        old_sec->length = new_end - old_sec->start;
                    } else {
                        old_sec->length = old_end - old_sec->start;
                    }
                    
                    // 不需要插入新的section，直接返回
                    return;
                }
            }
            
            // 不满足合并条件，将旧的section移动到下一层
            // 如果还有下一层
            if (level < gd_l2p_section[groupidx].depth - 1) {
                Insert(groupidx, *old_sec, level + 1);
            }
            
            // 删除当前层的这个section
            // 将最后一个元素移动到当前位置
            if (i < lvlsec->number - 1) {
                lvlsec->secgroup[i] = lvlsec->secgroup[lvlsec->number - 1];
            }
            lvlsec->number--;
            i--; // 重新检查当前位置的新元素
        }
    }
    
    // 没有冲突或处理完所有冲突后，插入新的section
    // 检查是否需要扩展数组
    // 这里假设secgroup有足够的空间，或者由调用者保证
    if (lvlsec->number >= 255) { // byte的最大值
        // 错误处理：层级已满
        return;
    }
    
    // 插入新的section
    lvlsec->secgroup[lvlsec->number] = sec;
    lvlsec->number++;
}

void fv_ftl_hdl(void)
{
    byte lb_rw_cnt = RWCMD_THR;
    byte lb_p2lp_act = 0;
    byte lb_p2lp_ptr = 0;
    byte lb_raid_act = 0;
    byte lb_cmd_end;
    uint32_t groupidx;
    byte groupoffset;
    byte bucketidx;
    section sec;
    word secnum;
    dwrd nowgroup;
    byte find;
    byte candelete;
    dwrd allocateppn[BUFFERSIZE];
    int i;
    int j;
    int k;
    int l;
#ifdef N4KA_EN
    byte lb_n4rd_vld;
    byte lb_sect_cnt;
    byte lb_sect_map;
    byte lb_n4rd_idx;
#endif
    word lw_cmd_blk;
    dwrd ld_buf_ptr;
    dwrd ld_cmd_padr;
#ifdef N4KA_EN
    dwrd ld_data_base;
    dwrd ld_meta_base;
    dwrd ld_data_src;
    dwrd ld_meta_src;
#endif

    //access host cmd, and handle raid_parity, bt_page, p2l_page, tbl_unit_flush
#ifdef N4KA_EN
    while((rs_host_cmd.sb_cmd_type != HCMD_INV) || rb_n4rd_vld)
#else
    while(rs_host_cmd.sb_cmd_type != HCMD_INV)
#endif
    {
#ifndef LGET_EN
        //rb_cdir_bas = (byte)rs_host_cmd.sd_cmd_lext;
        inhash=-1;
        rs_host_cmd.sd_cmd_padr = L2P_NULL;
        groupidx = rs_host_cmd.sd_cmd_padr/GROUPSIZE;
        groupoffset = rs_host_cmd.sd_cmd_padr%GROUPSIZE;
        bucketidx = groupoffset%GROUPSIZE;
        for(i=0;i<gd_l2p_section[groupidx].bucket[bucketidx].num;++i){
            if(gd_l2p_section[groupidx].bucket[bucketidx].kvgroup[i].key == groupoffset){
                inhash=i;
                rs_host_cmd.sd_cmd_padr = gd_l2p_section[groupidx].bucket[bucketidx].kvgroup[i].value;
                break;
            }
        }
        if(rs_host_cmd.sd_cmd_ladr == L2P_NULL){
            find=0;
            for(i=0;i<gd_l2p_section[groupidx].depth;++i){
                for(j=0;j<gd_l2p_section[groupidx].levelsecgroup[i].number;++j){
                    if((groupoffset>=gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].start)&&(groupoffset<=gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].start+gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].length)){
                        if((groupoffset-gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].start)%gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].step==0){
                            rs_host_cmd.sd_cmd_padr = gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].b + (groupoffset-gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].start)/gd_l2p_section[groupidx].levelsecgroup[i].secgroup[j].step;
                        }
                        find=1;
                        break;
                }
            }
            if(find==1){break;}
        }
    }
#endif

#ifdef N4KA_EN
        //check non 4KB align read queue
        lb_n4rd_vld = rb_n4rd_vld;
#endif

        //nand read cmd handle
        if(rs_host_cmd.sb_cmd_type == HCMD_RD)//read command
        {
#ifdef HWCMD_CHK
            fv_uart_print("r%x,%x,%x,%x,%x\r\n", (rs_host_cmd.sd_cmd_cid >> 7), rs_host_cmd.sd_cmd_ladr, rs_host_cmd.sb_cmd_ftl, rs_host_cmd.sb_cmd_crd, rs_host_cmd.sb_cmd_crl);
            if(rs_host_cmd.sb_cmd_ftl == 0)
            {
                rb_host_get = HCMD_INV;
                continue;
            }
#endif
            while(rw_cpua_set & CPUA_ACT);
            fv_nand_read();

            //get host cmd
            rb_host_get = HCMD_INV;
        }

        //nand write cmd handle
#ifdef N4KA_EN
        else if(lb_n4rd_vld || ((rs_host_cmd.sb_cmd_type == HCMD_WR) && ((rs_host_cmd.sb_cmd_smap == BYTE_MASK) ||
                                                                         (rs_host_cmd.sd_cmd_padr == L2P_NULL) ||
                                                                         (rs_host_cmd.sb_cmd_hit && fb_ploc_chk(rs_host_cmd.sd_cmd_padr)))))
        {
            if(lb_n4rd_vld)
            {
                rs_host_cmd.sb_cmd_mued = 1;
                rs_host_cmd.sb_cmd_strm = gb_n4rd_strm[rb_n4rd_idx];
                rs_host_cmd.sw_cmd_bfp  = rw_n4rd_bfp;
                rs_host_cmd.sd_cmd_ladr = gd_n4rd_ladr[rb_n4rd_idx];
                rs_host_cmd.sd_cmd_padr = gd_n4rd_padr[rb_n4rd_idx];
            }
            else if(rs_host_cmd.sb_cmd_smap != BYTE_MASK) //1. L2P_NULL, add dummy into data buffer, 2. gcwr hit, copy gcp data to bfp
            {
                rb_cawt_set = CAWT_ACT;

                if((rs_host_cmd.sw_cmd_bfp >> BPTR_SFT) == SMDT_BASE)
                {   
                    ld_data_base = SRAM_MEM_BASE  + ((dwrd)(rs_host_cmd.sw_cmd_bfp & BPTR_MSK) << 12);
                    ld_meta_base = SRAM_META_BASE + ((dwrd)(rs_host_cmd.sw_cmd_bfp & BPTR_MSK) << 12);
                }
                else
                {
                    ld_data_base = DRAM_DATA_BASE + ((dwrd)rs_host_cmd.sw_cmd_bfp << 12);
                    ld_meta_base = DRAM_META_BASE + ((dwrd)rs_host_cmd.sw_cmd_bfp << 8);
                }

                if((rs_host_cmd.sw_cmd_gcp >> BPTR_SFT) == SMDT_BASE)
                {   
                    ld_data_src = SRAM_MEM_BASE  + ((dwrd)(rs_host_cmd.sw_cmd_gcp & BPTR_MSK) << 12);
                    ld_meta_src = SRAM_META_BASE + ((dwrd)(rs_host_cmd.sw_cmd_gcp & BPTR_MSK) << 12);
                }
                else
                {
                    ld_data_src = DRAM_DATA_BASE + ((dwrd)rs_host_cmd.sw_cmd_gcp << 12);
                    ld_meta_src = DRAM_META_BASE + ((dwrd)rs_host_cmd.sw_cmd_gcp << 8);
                }

                while(rb_cawt_rdy == 0);
                lb_sect_map = (byte)(rd_stbl_dat(rs_host_cmd.sw_cmd_bfp) >> 1);

                for(lb_sect_cnt = 0; lb_sect_cnt < 8; lb_sect_cnt = lb_sect_cnt + 1)
                {
                    if((lb_sect_map & (1 << lb_sect_cnt)) == 0x0)
                    {
                        if(rs_host_cmd.sd_cmd_padr == L2P_NULL)
                        {
                            memset((byte *)(ld_data_base + (lb_sect_cnt << 9)), 0x0, 512);
                            *(volatile dwrd *)(ld_meta_base + (lb_sect_cnt << 3) + 0) = 0x4d558a87; //zero payload crc result
#ifdef DLNK_EN
                            *(volatile dwrd *)(ld_meta_base + (lb_sect_cnt << 3) + 4) = (rs_host_cmd.sd_cmd_ladr << (rb_dlnk_ena ? 4 : 3)) | ((rb_dlnk_ena && !rb_dlnk_mst) ? 0x8 : 0x0) | (dwrd)lb_sect_cnt;
#else
                            *(volatile dwrd *)(ld_meta_base + (lb_sect_cnt << 3) + 4) = (rs_host_cmd.sd_cmd_ladr << 3) | (dwrd)lb_sect_cnt;
#endif
                        }
                        else
                        {
                            memcpy((byte *)(ld_data_base + (lb_sect_cnt << 9)), (byte *)(ld_data_src + (lb_sect_cnt << 9)), 512);
                            memcpy((byte *)(ld_meta_base + (lb_sect_cnt << 3)), (byte *)(ld_meta_src + (lb_sect_cnt << 3)), 8);
                        }
                    }
                }

                rd_stbl_dat(rs_host_cmd.sw_cmd_bfp) = (BYTE_MASK << 1);
                rb_cawt_set = 0;
            }
#else
        else if(rs_host_cmd.sb_cmd_type == HCMD_WR)
        {
#endif

#ifdef HWCMD_CHK
#ifdef N4KA_EN
            if(lb_n4rd_vld)
            {
                fv_uart_print("nw%x,%x\r\n", rw_n4rd_bfp, rd_n4rd_idx);
            }
            else
#endif
            {
                fv_uart_print("w%x,%x,%x\r\n", (rs_host_cmd.sd_cmd_cid >> 7), rs_host_cmd.sd_cmd_ladr, rs_host_cmd.sb_cmd_mued);
                if(rs_host_cmd.sb_cmd_ftl == 0)
                {
                    rb_host_get = HCMD_INV;
                    continue;
                }
            }
#endif
            if(buffernum<BUFFERSIZE){
               dwrd mid;
               int low;
               int high;
               low=0;
               high=buffernum-1;
               while(low<high){
                  mid=(low+high)/2;
                  if(rs_host_cmd.sd_cmd_ladr>writebuffer[mid].lpn){
                     low=mid+1;
                  }else if(rs_host_cmd.sd_cmd_ladr<writebuffer[mid].lpn){
                     high=mid;
                  }else{
                     break;
                  }
               }
               for(i=buffernum;i>low;i--){
                  writebuffer[i]=writebuffer[i-1];
               }
               writebuffer[low].lpn=rs_host_cmd.sd_cmd_ladr;
               writebuffer[low].buf_ptr=rs_host_cmd.sw_cmd_bfp;
               writebuffer[low].ppn=rs_host_cmd.sd_cmd_padr;
               writebuffer[low].mued=rs_host_cmd.sb_cmd_mued;
               writebuffer[low].stream=rs_host_cmd.sb_cmd_strm;
               writebuffer[low].inhash=inhash;
               buffernum++;
            }
            else{
            
            memset(allocateppn,0,sizeof(allocateppn));
            for(i=0;i<buffernum;++i){
            //set head blk index for p4k_jump
            rb_head_idx = writebuffer[i].stream;//stream id(now always 0)
            ARM_NOP(); //wait 3 cycle for HW handle

            do
            {
                lb_cmd_end = (rb_frag_loc == ((FRAG_QNTY - 1)) && (rb_gwln_loc == rb_gwln_end)) ? 1 : 0;
                ld_cmd_padr = rd_hp4k_loc;//rd_hp4k_loc is the next valid ppn

                if(lb_p2lp_act) //p2l page
                {
                    ld_buf_ptr = (FTL_HTP2L_BASE >> 12) | (dwrd)lb_p2lp_ptr;
                    lb_p2lp_act = (lb_p2lp_ptr == 3) ? 0 : 1;
                    lb_p2lp_ptr = (lb_p2lp_ptr == 3) ? 0 : (lb_p2lp_ptr + 1);

                    rb_nddn_bit = 0xf; //clear transfer done flag
                    while(rb_ddma_act);

#ifdef CPUA_PROC
                    //enable cpua_proc
                    while(rw_cpua_set & CPUA_ACT);
                    rb_nblk_idx = rb_head_idx;
                    rd_cpua_reg0  = 0x0;
                    rd_cpua_reg1  = ld_cmd_padr & 0x3;
                    rd_cpua_reg3  = ld_buf_ptr & 0xfff;
                    rd_cpua_reg9  = ld_cmd_padr;
                    rd_cpua_reg10 = (ld_buf_ptr >> 12);
                    rd_cpua_reg11 = (QLNK_TRQ << 24) | ((dwrd)gb_htwl_sel << FRAG_SHIFT) | (dwrd)rb_frag_loc;
                    rd_cpua_reg12 = PROG_CPTR | LOW_PRI | CMD_BUF | (lb_cmd_end ? CMD_END : 0);
                    rw_cpua_set = CPUA_ACT | P2LP_CMD;
#else
                    //send program cmd to HW
                    fv_nand_htprg(lb_cmd_end, ld_buf_ptr, ld_cmd_padr);
#endif
                }
                else //host write cmd
                {
                    ld_buf_ptr = (dwrd)writebuffer[i].buf_ptr;

                    //original vcnt
                    if(writebuffer[i].ppn != L2P_NULL)//重写
                    {
#ifdef FTL_DBG
                        byte lb_vmap_ofs;
                        lb_vmap_ofs = (((byte)rs_host_cmd.sd_cmd_padr << CMPR_SHIFT) & 0x1f) | (gb_crl_tbl[rs_host_cmd.sd_cmd_ladr] & CMPR_MASK);
                        if((gd_vmap_tbl[(rs_host_cmd.sd_cmd_padr >> (5 - CMPR_SHIFT))] & (1 << lb_vmap_ofs)) == 0x0)
                        {
                            fv_uart_print("ht vld map table clr err: p4k:%x, vmap_ofs:%x\r\n", rs_host_cmd.sd_cmd_padr, lb_vmap_ofs);
                            fv_dbg_loop(0x2);
                        }
                        gd_vmap_tbl[(rs_host_cmd.sd_cmd_padr >> (5 - CMPR_SHIFT))] &= ~(1 << lb_vmap_ofs);
#endif
                        lw_cmd_blk = (word)(writebuffer[i].ppn >> (PAGE_SHIFT + BKCH_SHIFT + FRAG_SHIFT));
                        if(gd_vcnt_tbl[lw_cmd_blk] == 0)
                        {
                            fv_uart_print("host vld cnt table err: block:%x\r\n", lw_cmd_blk);
                            fv_dbg_loop(0x8);
                        }
                        gd_vcnt_tbl[lw_cmd_blk]--;//gd_vcnt_tbl表示了每个块中有效页的数量

                        if((gd_vcnt_tbl[lw_cmd_blk] == 0) && ((gb_blk_type[lw_cmd_blk] & HEAD_BLK) == 0x0))
                        {
                            if((gb_blk_type[lw_cmd_blk] & VCNT_BLK) == 0x0)
                            {
                                fv_uart_print("host_wr release block err: block: %d\r\n", lw_cmd_blk);
                                fv_dbg_loop(0x7);
                            }

                            gb_blk_type[lw_cmd_blk] &= ~VCNT_BLK;
                            if((gb_blk_type[lw_cmd_blk] & FREE_MASK) == FREE_BLK)
                            {
                                //update blk info
                                if(gw_fblk_str == DBLK_INVLD)
                                {
                                    gw_fblk_str = lw_cmd_blk;
                                }
                                else
                                {
                                    gs_dbl_tbl[gw_fblk_end].wd.w_next_ptr = lw_cmd_blk;
                                }
                                gw_fblk_end = lw_cmd_blk;
                                gw_fblk_num++;

                                fv_uart_print("host_wr release blk: blk_num:%x, free blk cnt:%d\r\n", lw_cmd_blk, gw_fblk_num);
                                if(gw_fblk_num == GC_BLK_THR)
                                {
                                    fv_uart_print("gc task finish: host\r\n");
                                }
                            }
                        }
                    }

                    //get free block
                    if((rb_head_stg == 0x0) && ((gw_htp2l_ofs & CMPR_MASK) == 0x0))
                    {
                        //check free block
                        if(gw_fblk_num == 0x0)
                        {
                            fv_uart_print("ht free blk zero\r\n");
                            fv_dbg_loop(0x1);
                        }

                        //update blk info
                        lw_cmd_blk = gw_fblk_str;
                        gw_fblk_str = (lw_cmd_blk == gw_fblk_end) ? DBLK_INVLD : gs_dbl_tbl[lw_cmd_blk].wd.w_next_ptr;
                        gb_blk_type[lw_cmd_blk] = HEAD_BLK;
                        gw_fblk_num--;
                        fv_uart_print("host_wr get new blk: blk_num:%x, free blk cnt:%d\r\n", lw_cmd_blk, gw_fblk_num);
                        if(gw_fblk_num == (GC_BLK_THR - 1))
                        {
                            fv_uart_print("gc task start: host\r\n");
                        }

                        rw_hblk_loc = lw_cmd_blk;
                        ld_cmd_padr = (dwrd)lw_cmd_blk << (PAGE_SHIFT + BKCH_SHIFT + FRAG_SHIFT);
#ifdef FTL_DBG
                        gd_htp2l_ptr = (dwrd)lw_cmd_blk << (PAGE_SHIFT + BKCH_SHIFT + FRAG_SHIFT + CMPR_SHIFT);
#endif

                        //send erase cmd to HW
                        while(rw_cpua_set & CPUA_ACT);
                        fv_nand_erase(lw_cmd_blk);
                        gd_ecnt_tbl[lw_cmd_blk]++;
                    }

                    //write l2p & vcnt table
                    
                    gd_vcnt_tbl[rw_hblk_loc]++;
#ifdef FTL_DBG
                    gb_crl_tbl[rs_host_cmd.sd_cmd_ladr] = (byte)gw_htp2l_ofs & CMPR_MASK;

                    {
                        byte lb_vmap_ofs;
                        lb_vmap_ofs = (((byte)ld_cmd_padr << CMPR_SHIFT) & 0x1f) | ((byte)gw_htp2l_ofs & CMPR_MASK);
                        if((gd_vmap_tbl[(ld_cmd_padr >> (5 - CMPR_SHIFT))] & (1 << lb_vmap_ofs)) != 0x0)
                        {
                            fv_uart_print("ht vld map table set err: p4k:%x, vmap_ofs:%x\r\n", ld_cmd_padr, lb_vmap_ofs);
                            fv_dbg_loop(0x3);
                        }
                        gd_vmap_tbl[(ld_cmd_padr >> (5 - CMPR_SHIFT))] |= (1 << lb_vmap_ofs);
                    }
#endif

#ifdef LGET_EN
                    rd_lupd_l4k = rs_host_cmd.sd_cmd_ladr;
                    rd_lupd_p4k = ld_cmd_padr;
                    rw_lupd_set = LUPD_ACT;
#endif

                    //set check zone
#ifdef N4KA_EN
                    if(lb_n4rd_vld == 0)
#endif
                    {
                        //rq_ochk_pat = (qwrd)rs_host_cmd.sd_cmd_padr;
                        rd_ochk_pat = writebuffer[i].ppn;
                    }

#ifdef FTL_DBG
                    if((gd_htp2l_ptr & 0xfff) != gw_htp2l_ofs)
                    {
                        fv_uart_print("ht p2l ptr error1: gd_htp2l_ptr:%x, gw_htp2l_ofs:%x\r\n", gd_htp2l_ptr, gw_htp2l_ofs);
                        fv_dbg_loop(0x4);
                    }

                    //write p2l table of dram for debug
                    gd_p2l_tbl[gd_htp2l_ptr] = rs_host_cmd.sd_cmd_ladr;
                    gd_htp2l_ptr++;
#endif

                    //write p2l tmp table
                    gd_htp2l_ladr[gw_htp2l_ofs & 0x3] = writebuffer[i].lpn;

#ifdef CPUA_PROC
                    //enable cpua_proc
                    while(rw_cpua_set & CPUA_ACT);
                    rb_nblk_idx = rb_head_idx;
                    rd_cpua_reg0  = 0x0;
                    rd_cpua_reg3  = ld_buf_ptr;
                    rd_cpua_reg4  = gd_htp2l_ladr[0];
                    rd_cpua_reg5  = gd_htp2l_ladr[1];
                    rd_cpua_reg6  = gd_htp2l_ladr[2];
                    rd_cpua_reg7  = gd_htp2l_ladr[3];
                    rd_cpua_reg8  = rs_host_cmd.sd_cmd_ladr;
                    rd_cpua_reg9  = ld_cmd_padr;
                    rd_cpua_reg10 = SRAM_MEM_BASE | ((dwrd)gw_htp2l_bfp << 12) | ((dwrd)gw_htp2l_ofs << 2);
                    rd_cpua_reg11 = (QLNK_HTQ << 24) | ((dwrd)gb_htwl_sel << FRAG_SHIFT) | rb_frag_loc;
                    rd_cpua_reg12 = PROG_CPTR | LOW_PRI | CMD_BUF | (lb_cmd_end ? CMD_END : 0);
                    rw_cpua_set = CPUA_ACT | DATA_CMD;
#else
                    //write l4k/p4k to fw meta
                    if((ld_buf_ptr >> BPTR_SFT) == SMDT_BASE)
                    {
                        rq_smmt_dat(ld_buf_ptr, (HMETA_QSZ+0)) = ((qwrd)ld_cmd_padr << DWRD_SHIFT) | (qwrd)writebuffer[i].lpn;
                        memset((byte *)(&rq_smmt_dat(ld_buf_ptr, (HMETA_QSZ+1))), 0, (4*6));
                    }
                    else
                    {
                        rq_dmmt_dat(ld_buf_ptr, (HMETA_QSZ+0)) = ((qwrd)ld_cmd_padr << DWRD_SHIFT) | (qwrd)writebuffer[i].lpn;
                        memset((byte *)(&rq_dmmt_dat(ld_buf_ptr, (HMETA_QSZ+1))), 0, (4*6));
                    }

                    //write p2l page to bmu sram
                    if((gw_htp2l_ofs & 0x3) == 0x3)
                    {
                        memcpy((byte *)(&rd_smdt_dat(gw_htp2l_bfp, (gw_htp2l_ofs & 0x3fc))), (byte *)(gd_htp2l_ladr), (4*4));
                    }

                    //send program cmd to HW
                    fv_nand_htprg(lb_cmd_end, ld_buf_ptr, ld_cmd_padr);
#endif
                    
                    //move p2l 4KB data from sram to dram
                    if((gw_htp2l_ofs & 0x3ff) == 0x3ff)
                    {
                        while(rw_cpua_set & CPUA_ACT);
                        while((rb_nddn_bit & 0xf) != 0xf); //check whether last p2l prgram done
                        while(rb_ddma_act);
                        rd_ddma_dad = ((FTL_HTP2L_BASE | ((gw_htp2l_ofs << 2) & 0x3000)) >> 4);
                        rd_ddma_sad = ((dwrd)gw_htp2l_bfp << (13 - 4));
                        rb_ddma_wen = 1;
                        rb_ddma_act = 1;
                        gw_htp2l_bfp = (gw_htp2l_bfp == 8) ? 9 : 8;

                        //move p2l page from dram to nand
                        if(gw_htp2l_ofs == 0xfff)
                        {
                            gw_htp2l_ofs = 0xffff; //set as -1
                            lb_p2lp_act = 1;
#ifdef GC_SIM
                            gb_htp2l_num++;
#endif
                        }
                    }

                    gw_htp2l_ofs++;
                } //if(lb_p2lp_act) else


                //set next padr
                lw_cmd_blk = rw_hblk_loc;
                rb_hp4k_set = JUMP_ACT;
                while(rb_hp4k_set & JUMP_ACT);
                if(rb_head_stg == 0x0)
                {
                    if(gd_vcnt_tbl[lw_cmd_blk] != 0)
                    {
                        gb_blk_type[lw_cmd_blk] = VCNT_BLK;
                    }
                    else
                    {
                        //update blk info
                        if(gw_fblk_str == DBLK_INVLD)
                        {
                            gw_fblk_str = lw_cmd_blk;
                        }
                        else
                        {
                            gs_dbl_tbl[gw_fblk_end].wd.w_next_ptr = lw_cmd_blk;
                        }
                        gw_fblk_end = lw_cmd_blk;
                        gw_fblk_num++;

                        fv_uart_print("host_wr release blk 2: blk_num:%x, free blk cnt:%d\r\n", lw_cmd_blk, gw_fblk_num);
                        if(gw_fblk_num == GC_BLK_THR)
                        {
                            fv_uart_print("gc task finish: host2\r\n");
                        }
                    }

                    gb_htwl_sel = 0;
                }
                else
                {
                    gb_htwl_sel = (rb_head_stg == 0x1) ? ((gb_htwl_sel < 4) ? 4 : 0) : ((gb_htwl_sel & 0x4) | rb_gwln_loc);
                }

                //update token
                if(rb_head_stg != 0x7)
                {
#ifdef GC_SIM
                    if((gb_htp2l_num > GC_SIM_THR) || (gd_ht_token < gd_gc_token))
#else
                    if((gw_fblk_num < GC_BLK_THR) || (gd_ht_token < gd_gc_token))
#endif
                    {
                        gd_ht_token = gd_ht_token + gd_ht_weight;
                    }
                }

                //move last p2l page from dram to nand
                if((rw_page_loc == (PAGE_QNTY - 1)) && (rb_cech_loc == (RDIE_QNTY - 1)) && (rb_frag_loc == (FRAG_QNTY - 4)))
                {
                    while(rw_cpua_set & CPUA_ACT);

                    //add dummy
                    while(gw_htp2l_ofs < 0x1000)
                    {
#ifdef FTL_DBG
                        if((gd_htp2l_ptr & 0xfff) != gw_htp2l_ofs)
                        {
                            fv_uart_print("ht p2l ptr error2: gd_htp2l_ptr:%x, gw_htp2l_ofs:%x\r\n", gd_htp2l_ptr, gw_htp2l_ofs);
                            fv_dbg_loop(0x5);
                        }

                        //write p2l table of dram for debug
                        gd_p2l_tbl[gd_htp2l_ptr] = P2L_NULL;
                        gd_htp2l_ptr++;
#endif

                        rd_smdt_dat(gw_htp2l_bfp, (gw_htp2l_ofs & 0x3ff)) = P2L_NULL;

                        //move p2l 4KB data from sram to dram
                        if((gw_htp2l_ofs & 0x3ff) == 0x3ff)
                        {
                            while((rb_nddn_bit & 0xf) != 0xf); //check whether last p2l program done
                            while(rb_ddma_act);
                            rd_ddma_dad = ((FTL_HTP2L_BASE | ((gw_htp2l_ofs << 2) & 0x3000)) >> 4);
                            rd_ddma_sad = ((dwrd)gw_htp2l_bfp << (13 - 4));
                            rb_ddma_wen = 1;
                            rb_ddma_act = 1;
                            gw_htp2l_bfp = (gw_htp2l_bfp == 8) ? 9 : 8;
                        }
                        gw_htp2l_ofs++;
                    }

                    //move p2l page from dram to nand
                    gw_htp2l_ofs = 0;
                    lb_p2lp_act = 1;
                }
            } while(lb_p2lp_act || lb_raid_act);

            //get host cmd
#ifdef N4KA_EN
            if(lb_n4rd_vld)
            {
                gb_n4rd_idxq[gb_n4rd_wptr] = rb_n4rd_idx;
                gb_n4rd_wptr = (gb_n4rd_wptr == 254) ? 0 : (gb_n4rd_wptr + 1);
                rb_n4rd_vld = N4RD_CLR;
                rb_host_get = LCMD_INV;
            }
            else
#endif
            {
                rb_host_get = HCMD_INV;
            }

            //check read unit cnt
            if(writebuffer[i].mued && (rb_frag_loc == 0x0))
            {
                break;
            }
        allocateppn[i] = ld_cmd_padr;
        } 
    
    secnum=0;
    for(i=0;i<=buffernum;++i){
        if(secnum==0){
            sec.start=(byte)(writebuffer[i].lpn%GROUPSIZE);
            sec.b=allocateppn[i];
            nowgroup=writebuffer[i].lpn/GROUPSIZE;
        }
        else{
            if(i==buffernum||((i<buffernum)&&((writebuffer[i].lpn/GROUPSIZE)!=nowgroup))){
                if(secnum==1){
                    if(writebuffer[i-1].inhash==-1){
                        gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[gd_l2p_section[nowgroup].bucket[sec.start%16].num].key=sec.start;
                        gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[gd_l2p_section[nowgroup].bucket[sec.start%16].num].value=sec.b;
                        gd_l2p_section[nowgroup].bucket[sec.start%16].num++;
                    }
                    else{
                        gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[writebuffer[i-1].inhash].key=sec.start;
                        gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[writebuffer[i-1].inhash].value=sec.b;
                    }
                }
                else{
                    Insert(nowgroup,sec,0);
                    for(j=0;j<=sec.length/sec.step;++j){
                        if(writebuffer[i-j-1].inhash!=-1){
                            for(k=writebuffer[i].inhash;k<gd_l2p_section[nowgroup].bucket[sec.start%16].num-1;++k){
                                gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[k]=gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[k+1];
                            }
                            gd_l2p_section[nowgroup].bucket[sec.start%16].num--;
                        }
                    }
                }
                if(i<buffernum){
                sec.length=0;
                sec.step=0;
                sec.start=writebuffer[i].lpn;
                sec.b=allocateppn[i];
                secnum=1;
                nowgroup=writebuffer[i].lpn/GROUPSIZE;
            }
            }
            else{
                if(secnum==1){sec.step=(byte)((writebuffer[i].lpn%GROUPSIZE)-sec.start);sec.length=(byte)((writebuffer[i].lpn%GROUPSIZE)-sec.start);secnum++;}
                else{
                    if(((writebuffer[i].lpn%GROUPSIZE)-sec.start-sec.length!=sec.step)||(allocateppn[i]-(sec.b+sec.length/sec.step))!=1){
                       Insert(nowgroup,sec,0);
                    for(j=0;j<=sec.length/sec.step;++j){
                        if(writebuffer[i-j-1].inhash!=-1){
                            for(k=writebuffer[i].inhash;k<gd_l2p_section[nowgroup].bucket[sec.start%16].num-1;++k){
                                gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[k]=gd_l2p_section[nowgroup].bucket[sec.start%16].kvgroup[k+1];
                            }
                            gd_l2p_section[nowgroup].bucket[sec.start%16].num--;
                        }
                    }
                
                
                sec.length=0;
                sec.step=0;
                sec.start=writebuffer[i].lpn;
                sec.b=allocateppn[i];
                secnum=1;
                }
            
                    
                else{
                     sec.length=(byte)((writebuffer[i].lpn%GROUPSIZE)-sec.start);secnum++;
                    }
                }
            }
        }
    }
    writeinit+=buffernum;
    if(writeinit>THRESHOLD){
      for(i=0;i<GROUPNUM;++i){
        if(gd_l2p_section[i].depth==0){continue;}
        memset(appear,0,sizeof(appear));
        for(j=0;j<gd_l2p_section[i].depth;++j){
            k=0;
            while(1){
                if(k>=gd_l2p_section[i].levelsecgroup[j].number){break;}
                candelete=1;
               for(l=0;l<=(gd_l2p_section[i].levelsecgroup[j].secgroup[k].length/gd_l2p_section[i].levelsecgroup[j].secgroup[k].step);++l){
                if(appear[(gd_l2p_section[i].levelsecgroup[j].secgroup[k].b+l*gd_l2p_section[i].levelsecgroup[j].secgroup[k].step)]==0){
                    candelete=0;
                    appear[(gd_l2p_section[i].levelsecgroup[j].secgroup[k].b+l*gd_l2p_section[i].levelsecgroup[j].secgroup[k].step)]=1;}
                    
                }
                if(candelete==1){
                    for(l=k;l<gd_l2p_section[i].levelsecgroup[j].number-1;++l){
                        gd_l2p_section[i].levelsecgroup[j].secgroup[l]=gd_l2p_section[i].levelsecgroup[j].secgroup[l+1];
                    }
                    gd_l2p_section[i].levelsecgroup[j].number--;
                }
                k++;
            }
        }
      }
    }
    buffernum=0;
    }//else if(rs_host_cmd.sb_cmd_type == HCMD_WR)
    }
#ifdef N4KA_EN
        //nand write non 4KB cmd handle
        else if((rs_host_cmd.sb_cmd_type == HCMD_WR) && (gb_n4rd_rptr != gb_n4rd_wptr))
        {
#ifdef HWCMD_CHK
            fv_uart_print("nr%x,%x,%x\r\n", (rs_host_cmd.sd_cmd_cid >> 7), rs_host_cmd.sd_cmd_ladr);
            if(rs_host_cmd.sb_cmd_ftl == 0)
            {
                rb_host_get = HCMD_INV;
                continue;
            }
#endif
            lb_n4rd_idx = gb_n4rd_idxq[gb_n4rd_rptr];
            gb_n4rd_rptr = (gb_n4rd_rptr == 254) ? 0 : (gb_n4rd_rptr + 1);
            gb_n4rd_strm[lb_n4rd_idx] = rs_host_cmd.sb_cmd_strm;
            gd_n4rd_ladr[lb_n4rd_idx] = rs_host_cmd.sd_cmd_ladr;
            gd_n4rd_padr[lb_n4rd_idx] = rs_host_cmd.sd_cmd_padr;

            //set check zone
            //rq_ochk_pat = (qwrd)rs_host_cmd.sd_cmd_padr;
            rd_ochk_pat = rs_host_cmd.sd_cmd_padr;

            while(rw_cpua_set & CPUA_ACT);
            fv_nand_n4k(lb_n4rd_idx, rs_host_cmd.sb_cmd_hit);

            //get host cmd
            rb_host_get = HCMD_INV;
        }
#endif

        //other cmd handle, trim, flush, power mode, security erase, download FW, system info
        else
        {
            fv_ftl_oth();

            //get host cmd
            rb_host_get = HCMD_INV;

            break;
        }

        //update Read/Write command Cnt
        lb_rw_cnt--;
        if(lb_rw_cnt == 0x0)
        {
            break;
        }
    } //while(rs_host_cmd.sb_cmd_type != HCMD_INV)

//    //pre read check
//    if(rb_bmu_hold)
//    {
//        //if pre_read tbl has this LBA, enable nau and send to host
//        //else if pre read end, skip this data
//        //else record to pre_read tbl
//    }

    return;
}
