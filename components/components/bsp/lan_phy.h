#define PHY_RESET_CONTROL_REG           (0x0)
#define SOFTWARE_RESET                     BIT(15)
#define SOFTWARE_AUTO_NEGTIATION           BIT(12)

#define BASIC_MODE_STATUS_REG           (0x1)
#define AUTO_NEGOTIATION_COMPLETE          BIT(5)
#define LINK_STATUS                        BIT(2)

#define AUTO_NEG_ADVERTISEMENT_REG      (0x4)
#define ASM_DIR                            BIT(11)
#define PAUSE                              BIT(10)

#define PHY_LINK_PARTNER_ABILITY_REG    (0x5)
#define PARTNER_ASM_DIR                    BIT(11)
#define PARTNER_PAUSE                      BIT(10) 

#define PHY_STATUS_REG                  (0x1f)
#define AUTO_NEGTIATION_STATUS             BIT(12)
#define DUPLEX_STATUS                      BIT(4)
#define SPEED_STATUS                       BIT(2)       

