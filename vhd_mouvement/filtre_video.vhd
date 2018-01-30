library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity filtre_video is
	generic (
		size	: integer := 9 	-- taille de la sous-fenetre = 2**size pixels
		);
    	port (
		--horloge et reset
		CLK			: in std_logic; -- clock ï¿½ 54 MHz
		RESET 		: in std_logic; -- reset ï¿½ 0 			
		-- flux video ï¿½ 27 MHz
		-- synchro
		VGA_X :	in std_logic_vector(10 downto 0); -- compteur pixels
		VGA_Y :	in std_logic_vector(10 downto 0); -- compteur lignes
		-- entrï¿½e
		iY : 	in std_logic_vector(7 downto 0); -- flux video entrant : luminance
		iCb : 	in std_logic_vector(7 downto 0); -- flux video entrant : chrominance bleu
		iCr : 	in std_logic_vector(7 downto 0); -- flux video entrant : chrominance rouge
		-- sortie
		oY	: 	out std_logic_vector(7 downto 0); -- flux video sortant : luminance
		oCb	: 	out std_logic_vector(7 downto 0); -- flux video sortant : chrominance bleu
		oCr	: 	out std_logic_vector(7 downto 0); -- flux video sortant : chrominance rouge
		--switch D2E
		switch			: in std_logic_vector(17 downto 0);		-- ï¿½ connecter ï¿½ DPDT_SW;
		-- SRAM interfaces
		address_SRAM 	: out std_logic_vector(17 downto 0); 	-- ï¿½ connecter ï¿½ SRAM_ADDR
		data_SRAM		: inout std_logic_vector(15 downto 0);  -- ï¿½ connecter ï¿½ SRAM_DQ
		write_enable 	: out std_logic; 						-- ï¿½ connecter ï¿½ SRAM_WE_N
		read_enable 	: out std_logic; 						-- ï¿½ connecter ï¿½ SRAM_OE_N
		chip_enable 	: out std_logic;						-- ï¿½ connecter ï¿½ SRAM_CE_N 
		high_mask 		: out std_logic ; 						-- ï¿½ connecter ï¿½ SRAM_UB_N
		low_mask 		: out std_logic 			
		);			
end entity filtre_video;


architecture A of filtre_video is

component memoire_ligne 
		generic (
		address_size : integer;
		word_size : integer
		);
	    port (
		CLK			: in std_logic;
		RESET		: in std_logic;		
		address 	: in std_logic_vector(address_size-1 downto 0);
		data_in		: in std_logic_vector(word_size-1 downto 0);
		data_out	: out std_logic_vector(word_size-1 downto 0);
		read_write	: in std_logic
		);
end component ;


component module_fenetrage -- Sort un signal in_active_area
	generic (
			size			: integer := 8
			);
	port (
			VGA_X 			:	in std_logic_vector(10 downto 0);
			VGA_Y 			:	in std_logic_vector(10 downto 0);			
			iY 				: 	in std_logic_vector(7 downto 0);
			oY				: 	out std_logic_vector(7 downto 0);
			in_active_area 	:	out std_logic; -- Passe ï¿½ 1 pendant 521 pixs et revient ï¿½ 0(c'est un espï¿½ce d'enable du process).
			X_cpt			:	out std_logic_vector(10 downto 0);
			Y_cpt			:	out std_logic_vector(10 downto 0)
		);
end component;


-- component module_SRAM -- On va pas se servir de ce module du coup on le commente. 
--     port (
-- 			CLK				: 	in std_logic;
-- 			RESET 			: 	in std_logic;			 
-- 			--module top interface
-- 			in_active_area	:	in std_logic;
-- 			X_cpt			: 	in std_logic_vector(10 downto 0);
-- 			Y_cpt			: 	in std_logic_vector(10 downto 0);
-- 			data_in			:	in std_logic_vector(7 downto 0);
-- 			data_out		: 	out std_logic_vector(7 downto 0);					
-- 			-- SRAM interfaces
-- 			address_SRAM 	: out std_logic_vector(17 downto 0); 	-- ï¿½ connecter ï¿½ SRAM_ADDR
-- 			data_SRAM		: inout std_logic_vector(15 downto 0);  -- ï¿½ connecter ï¿½ SRAM_DQ composant physique datasram
-- 			write_enable 	: out std_logic; 						-- ï¿½ connecter ï¿½ SRAM_WE_N en quinconce avec read_enable (signaal read_write)
-- 			read_enable 	: out std_logic; 						-- ï¿½ connecter ï¿½ SRAM_OE_N
-- 			chip_enable 	: out std_logic;						-- ï¿½ connecter ï¿½ SRAM_CE_N   en permanence 0
-- 			high_mask 		: out std_logic ; 						-- ï¿½ connecter ï¿½ SRAM_UB_N 0
-- 			low_mask 		: out std_logic  						-- ï¿½ connecter ï¿½ SRAM_LB_N 0
-- 			);			
-- end component;


component module_diff -- C'est celui qui fait notre process.
  port (
	in_active_area 		: 	in 	std_logic ;
	iY1					:	in	std_logic_vector(7 downto 0) ;
	iY2					:	in	std_logic_vector(7 downto 0) ;
	oY					: 	out	std_logic_vector(7 downto 0) ;
	threshold			: 	in 	std_logic_vector(7 downto 0) 
  ) ;
end component ; -- module_diff

--signaux flux video
signal sig_Y1			: std_logic_vector(7 downto 0) ;
signal sig_Y2			: std_logic_vector(7 downto 0) ;
signal sig_Y3			: std_logic_vector(7 downto 0) ;
signal sig_Y4			: std_logic_vector(7 downto 0) ;
signal sig_concat : std_logic_vector(17 downto 0);
signal mem2_vers_mem3 : std_logic_vector(17 downto 0);
signal sortie_finale : std_logic_vector(17 downto 0);
signal entree_mem1 : std_logic_vector(7 downto 0) ;
signal entree_mem2 : std_logic_vector(7 downto 0) ;
signal sortie_mem1 : std_logic_vector(7 downto 0);
signal sortie_mem2 : std_logic_vector(7 downto 0);
signal entree_mem1_v : std_logic_vector(7 downto 0);
signal entree_mem2_v : std_logic_vector(7 downto 0);
signal sortie_mem1_v : std_logic_vector(7 downto 0);
signal sortie_mem2_v : std_logic_vector(7 downto 0);


--signaux de synchro module fentrage
signal Y_cpt			: std_logic_vector(10 downto 0);
signal X_cpt 			: std_logic_vector(10 downto 0);
signal in_active_area 		: std_logic;

--signaux debug
signal threshold		: std_logic_vector(7 downto 0) ;

--signuax intermï¿½diares
signal address : std_logic_vector( size -1 downto 0) := (others => '0');
signal address_mem1 : std_logic_vector( 7 downto 0) := (others => '0');
signal address_mem2 : std_logic_vector( 7 downto 0) := (others => '1');
signal adr_SRAM :  std_logic_vector( 15 downto 0) := (others => '0');
signal read_write		:std_logic;
--signal chip_enable		:std_logic := '0';
--signal high_mask		:std_logic := '0';
--signal low_mask		:std_logic := '0';
signal SRAM1 : std_logic_vector (7 downto 0):= (others => '0');
signal SRAM2 : std_logic_vector (7 downto 0):= (others => '0');
signal SRAM1_v : std_logic_vector (7 downto 0):= (others => '0');
signal SRAM2_v : std_logic_vector (7 downto 0):= (others => '0');


begin
	u_1: module_fenetrage 
	generic map(
			size => size
			)
	port map(
			VGA_X => VGA_X,
			VGA_Y => VGA_Y,			
			iY	=> iY,
			oY	=> sig_Y1,
			in_active_area => in_active_area,
			X_cpt => X_cpt,
			Y_cpt => Y_cpt
			);
			
	mem1 :memoire_ligne 
		generic map (
			address_size => size,
			word_size => 8 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address,		
			data_in		=> data_SRAM(15 downto 8),
			data_out	=> sig_Y2,
			read_write	=> read_write
			);

	mem2 :memoire_ligne 
		generic map (
			address_size => size,
			word_size => 18 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address,
			data_in		=> sig_concat,
			data_out	=> mem2_vers_mem3,
			read_write	=> read_write
			);
	mem3 :memoire_ligne 
		generic map (
			address_size => size,
			word_size => 18 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address,
			data_in		=> mem2_vers_mem3,
			data_out	=> sortie_finale,
			read_write	=> read_write
			);
			
	mem1_lissage :memoire_ligne 
		generic map (
			address_size => 8,
			word_size => 8 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address_mem1,
			data_in		=> entree_mem1, 
			data_out	=> sortie_mem1,
			read_write	=> read_write
			);
			
	mem2_lissage :memoire_ligne 
		generic map (
			address_size => 8,
			word_size => 8 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address_mem2,
			data_in		=> entree_mem2,
			data_out	=> sortie_mem2,
			read_write	=> read_write
			);
			
		mem1_lissage_v :memoire_ligne 
		generic map (
			address_size => 8,
			word_size => 8 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address_mem1,
			data_in		=> entree_mem1_v, 
			data_out	=> sortie_mem1_v,
			read_write	=> read_write
			);
			
	mem2_lissage_v :memoire_ligne 
		generic map (
			address_size => 8,
			word_size => 8 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address_mem2,
			data_in		=> entree_mem2_v,
			data_out	=> sortie_mem2_v,
			read_write	=> read_write
			);

  address_SRAM <= "00"&adr_SRAM;
  lissage_gradient : process (clk)
  
  constant gamma1 : std_logic_vector(2 downto 0) := "001"; -- = 8*(1-gamma)
  constant gamma2 : std_logic_vector(2 downto 0) := "111"; -- = 8* gamma = 0.875
  variable reg_direct : std_logic_vector(7 downto 0) := (others => '0');
  variable reg_indirect : std_logic_vector(7 downto 0) := (others => '0');
  variable reg_direct_v : std_logic_vector(7 downto 0) := (others => '0');
  variable reg_indirect_v : std_logic_vector(7 downto 0) := (others => '0');
  variable compare : std_logic_vector( 7 downto 0) := "11111110" ;--(others => '1');
  variable compare_SRAM : std_logic_vector( 15 downto 0) := "1111111111111110";--(others => '1');
  variable sens : boolean := false;
  variable intermed : std_logic_vector(10 downto 0) := (others => '0');
  variable intermed_v : std_logic_vector(10 downto 0) := (others => '0');
  
  variable sens_sram : boolean := false;
  

  variable reg2: std_logic_vector(7 downto 0) := (others => '0');
	variable data_in: std_logic_vector(7 downto 0):= (others => '0');
	variable Gx: signed(8 downto 0) := (others => '0');
	variable Gy: signed(8 downto 0) := (others => '0');
	variable G: signed(17 downto 0) := (others => '0');
	variable orientation: std_logic_vector(1 downto 0) := (others => '0');
	
	
	variable regA: std_logic_vector(15 downto 0) := (others => '0');
	variable regB: std_logic_vector(17 downto 0) := (others => '0');
	-- regB plus grand besoin de son orientation (pixel central)
	variable regC: std_logic_vector(15 downto 0) := (others => '0');
	variable regD: std_logic_vector(15 downto 0) := (others => '0');
	variable regE: std_logic_vector(15 downto 0) := (others => '0');
	variable regF: std_logic_vector(15 downto 0) := (others => '0');
  begin
    
  if(clk = '1' and clk'event) then
  
  if(RESET = '1') then
    
    if (in_active_area = '1') then
      --write_enable <= read_enable;
      --read_enable <= (not) read_enable;
      read_write <= not read_write;
			if read_write = '1' then
			  
        if sens_SRAM = false then
          adr_SRAM <= std_logic_vector(unsigned(adr_SRAM) + 1) ;
          
          if adr_SRAM = compare_SRAM then
            sens_SRAM := not sens_SRAM; 
            adr_SRAM <= "0000000000000000"; 
          end if;
        else 
          adr_SRAM(15 downto 8) <=  std_logic_vector(unsigned(adr_SRAM(15 downto 8)) + 1) ;
          if adr_SRAM = compare_SRAM then
            sens_SRAM := not sens_SRAM; 
            adr_SRAM <= "0000000000000000"; 
          end if;
          if adr_SRAM(15 downto 8) = "11111111" then
            adr_SRAM <= "00000000"&std_logic_vector(unsigned(adr_SRAM(7 downto 0)) + 1) ;  
          end if;      
        end if;
        
      -- lissage 
      if (sens = false) then

        --lissage direct horizontal
        intermed := std_logic_vector((unsigned(gamma1)*unsigned(iY) + unsigned(gamma2)*unsigned(reg_direct))/8);
        entree_mem1 <= intermed(7 downto 0);
        --lissage direct vertical
        intermed_v := std_logic_vector((unsigned(gamma1)*unsigned(SRAM1_v) + unsigned(gamma2)*unsigned(reg_direct_v))/8);
        entree_mem1_v <= intermed_v(7 downto 0);
        
        address_mem1 <= std_logic_vector(unsigned(address_mem1) + 1) ;
        
        
        --lissage indirect horizontal
        intermed := std_logic_vector((unsigned(gamma1)*unsigned(sortie_mem2) + unsigned(gamma2)*unsigned(reg_indirect))/8);
        SRAM1 <= intermed(7 downto 0);   
        --lissage indirect vertical
        intermed_v := std_logic_vector((unsigned(gamma1)*unsigned(sortie_mem2_v) + unsigned(gamma2)*unsigned(reg_indirect_v))/8);
        SRAM1_v <= intermed_v(7 downto 0);
        
        address_mem2 <= std_logic_vector(unsigned(address_mem2) - 1) ;
        
        reg_indirect := sortie_mem2;
        reg_indirect_v := sortie_mem2_v;
    
        if(address_mem1 =  compare ) then
          sens := not sens;
        end if;
              
    
      elsif (sens = true) then

        --lissage direct horizontal
        intermed := std_logic_vector((unsigned(gamma1)*unsigned(iY) + unsigned(gamma2)*unsigned(reg_direct))/8);
        entree_mem2 <= intermed(7 downto 0);
        --lissage direct vertical
        intermed_v := std_logic_vector((unsigned(gamma1)*unsigned(SRAM1_v) + unsigned(gamma2)*unsigned(reg_direct_v))/8);
        entree_mem2_v <= intermed_v(7 downto 0);
        
        address_mem2 <= std_logic_vector(unsigned(address_mem2) + 1) ;
        
        
        --lissage indirect horizontal
        intermed := std_logic_vector((unsigned(gamma1)*unsigned(sortie_mem1) + unsigned(gamma2)*unsigned(reg_indirect))/8);
        SRAM1 <= intermed(7 downto 0);
        --lissage indirect vertical
        intermed_v := std_logic_vector((unsigned(gamma1)*unsigned(sortie_mem1_v) + unsigned(gamma2)*unsigned(reg_indirect_v))/8);
        SRAM1_v <= intermed_v(7 downto 0);
        
        address_mem1 <= std_logic_vector(unsigned(address_mem1) - 1) ;
        
        reg_indirect_v := sortie_mem1_v;
  
        if(address_mem2 = compare ) then
          sens := not sens;
        end if;
                      
      end if;
      --SRAM1 et SRAM2 transvasées dans data_SRAM (poids faibles et forts)      
      
      

      
      
      -- rotation des memoires
      reg_direct := iY;
      reg_direct_v := SRAM1_v ;
      --implémenation SRAM

      data_SRAM(7 downto 0) <= SRAM1;
      data_SRAM(15 downto 8) <= SRAM1_v;
    
      
      
      
      				
			--implémenattion gradient				  
					Gy := -signed('0'&SRAM2) + signed('0'&sig_Y2) -signed('0'&data_in) + signed('0'&reg2); 
					Gx := -signed('0'&SRAM2) - signed('0'&sig_Y2) +signed('0'&data_in) + signed('0'&reg2);
					G := Gx*Gx + Gy*Gy;
					address <= std_logic_vector(unsigned(address) + 1) ;

					if Gx*Gx > 4 * Gy*Gy then
						orientation := "10";
					elsif Gy*Gy > 4* Gx*Gx then
						orientation := "00";
					elsif Gx*Gy > 0 then
						orientation := "01";
					else
						orientation := "11";

					end if;
					
					G := G(17 downto 2)&signed(orientation);
					sig_concat <= std_logic_vector(G) ;
					
					-- Comparaison des pixels selon une direction.
					-- On travaille avec l'orientation du pixel 5 donc celui dans registre B.
					
					if regB(1 downto 0) = "00" and regB(17 downto 2)>= regA and regB(17 downto 2) >= regC then --verticale 6,4
					 
						 sig_Y3 <= (others => '1' );--std_logic_vector(G(17 downto 10)) ;
						 
						 
					elsif regB(1 downto 0) = "01" then -- 1 et 9  deuxiï¿½me bissectrice
					  if regB(17 downto 2) >= regD and regB(17 downto 2) >= sig_concat(17 downto 2) then
						 sig_Y3 <= (others => '1' );--std_logic_vector(G(17 downto 2))  ;
						 end if ;
						 
					elsif regB(1 downto 0) = "10" then -- horizontale 8,2
					  if regB(17 downto 2)>= regE and regB(17 downto 2)>= mem2_vers_mem3(17 downto 2) then
						 sig_Y3 <= (others => '1' );--std_logic_vector(G(17 downto 2))  ;
						 end if ;
						 
					elsif regB(1 downto 0) = "11" then-- permiï¿½re bissectrice 7,3
					  if regB(17 downto 2)>= regF and regB(17 downto 2)>= sortie_finale(17 downto 2) then
						 sig_Y3 <= (others => '1' ); --std_logic_vector(G(17 downto 2))  ;
						 end if;
						 
					else 
					  sig_Y3 <=(others => '0')  ;
					  
				  end if;
							 
					  
					  
					

				--end if; --pas sur peut etre a bouger après le endif de inactive area
				reg2 := sig_Y2;
				data_in := SRAM2;
				--Mise à jour des regitres
				
				-- On mets à jour les registres qui stockent les vals + anciennes
				-- Pour éviter des les perdres
				regF := regC;
				regE := regB(17 downto 2);
				regD := regA;
				
				--Registre les plus rï¿½cents qui stockent signaux qu'on vient de d'ï¿½mettre.
				regC := sig_concat(17 downto 2);
				regA := sortie_finale(17 downto 2);
				regB := mem2_vers_mem3;
				
				
			else
				read_write <= '1';
				read_enable <= '0';
				write_enable <= '1';
				
				SRAM1_v <= data_SRAM(7 downto 0);
				SRAM2 <= data_SRAM(15 downto 8);
			end if; --if read_write 
      
    else

    compare := "11111110" ;--(others => '1');
    intermed := (others => '0');
    intermed_v := (others => '0');
	  orientation := (others => '0');
		
      
      
    end if; --if inactive_area
    
  else
    reg_direct := (others => '0');
    reg_indirect := (others => '0');
    reg_direct_v := (others => '0');
    reg_indirect_v := (others => '0');
    compare := "11111110" ;--(others => '1');
    sens := false;
    sens_SRAM := false;
    intermed := (others => '0');
    intermed_v := (others => '0');

    reg2 := (others => '0');
	  data_in := (others => '0');
	  Gx := (others => '0');
	  Gy := (others => '0');
	  G := (others => '0');
	  orientation := (others => '0');
		
	  regA := (others => '0');
	  regB := (others => '0');
	  regC := (others => '0');
	  regD := (others => '0');
	  regE := (others => '0');
	  regF := (others => '0');


 
  end if; --if reset
  end if; -- if clk
  
  end process;

  



	--concurent
	threshold <= switch(17 downto 10);
	oCb <= X"80";		
	oCr <= X"80";

	--process
	process_affichage : process( switch, iY, sig_Y1, sig_Y2, sig_Y3)
	begin
		case( switch(4 downto 0) ) is
			when "00000" => oY <= iY; -- avant fenetrage		
			when "00001" => oY <= sig_Y1; -- aprï¿½s fenetrage				
			when "00011" => oY <= sig_Y2; -- aprï¿½s memoire
			when others  => oY <= sig_Y3;  -- aprï¿½s grad
			  --faire switch lissage
		end case ;
	end process ; -- process_affichage

end architecture A;	
