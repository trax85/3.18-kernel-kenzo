#ifndef _NF_DEFRAG_IPV6_H
#define _NF_DEFRAG_IPV6_H

void nf_defrag_ipv6_enable(void);

<<<<<<< HEAD
int nf_ct_frag6_init(void);
void nf_ct_frag6_cleanup(void);
struct sk_buff *nf_ct_frag6_gather(struct sk_buff *skb, u32 user);
void nf_ct_frag6_consume_orig(struct sk_buff *skb);
=======
extern int nf_ct_frag6_init(void);
extern void nf_ct_frag6_cleanup(void);
extern struct sk_buff *nf_ct_frag6_gather(struct sk_buff *skb, u32 user);
extern void nf_ct_frag6_consume_orig(struct sk_buff *skb);
>>>>>>> p9x

struct inet_frags_ctl;

#endif /* _NF_DEFRAG_IPV6_H */
