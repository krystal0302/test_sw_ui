import React, { createContext } from 'react'
import { useQuery } from 'react-query';
import { fetchGetToken } from '../api/FetchApi';

const TokenContext = createContext(null);

function TokenContextProvider({ children }) {
	const { data: token, error: errToken } = useQuery(
		['token'],
		fetchGetToken,
		{
			refetchOnWindowFocus: false,
			refetchOnmount: false,
			refetchOnReconnect: false,
			retry: false,
		}
	);
	return (
		<TokenContext.Provider value={{ token }}>
			{children}
		</TokenContext.Provider>
	)
}

export { TokenContext, TokenContextProvider }